/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include "common.h"

using namespace std;

class ImuGrabber : public rclcpp::Node
{
public:
    ImuGrabber() : rclcpp::Node("imu_grabber")
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    // Variables
    std::mutex mBufMutex;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(std::shared_ptr<ImuGrabber> imuGrabber)
        : Node("image_grabber"),
          mpImuGb(std::move(imuGrabber))
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    // Variables
    std::mutex mBufMutex;
    std::atomic<bool> mustStop{false};
    std::shared_ptr<ImuGrabber> mpImuGb;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> imgPCBuf;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgRGBBuf, imgDBuf;

    void SyncWithImu();
    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs);
    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr &msgD,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC);
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] Problem occured while running `cv_bridge`: %s", e.what());
    }

    return cv_ptr->image.clone();
}

/**
 * @brief Callback function to get scene segmentation results from the SemanticSegmenter module
 *
 * @param msgSegImage The segmentation results from the SemanticSegmenter
 */
void ImageGrabber::GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage)
{
    // Fetch the segmentation results
    cv_bridge::CvImageConstPtr cv_imgSeg;
    uint64_t key_frame_id = msgSegImage.key_frame_id.data;

    try
    {
        cv_imgSeg = cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(msgSegImage.segmented_image),
                                        sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "[Error] `cv_bridge` exception: %s", e.what());
        return;
    }

    // Convert to PCL PointCloud2 from `sensor_msgs` PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmented_image_probability, *pclPc2SegPrb);

    // Create the tuple to be appended to the segmentedImageBuffer
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(key_frame_id, cv_imgSeg->image, pclPc2SegPrb);

    // Add the segmented image to a buffer to be processed in the SemanticSegmentation thread
    pSLAM->addSegmentedImage(&tuple);
}

void ImageGrabber::SyncWithImu()
{
    if (!mpImuGb)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] IMU Grabber not initialized!");
        return;
    }

    while (!mustStop)
    {
        // Make sure thread synchronization is properly handled
        bool hasData = false;
        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            std::lock_guard<std::mutex> lock2(mpImuGb->mBufMutex);
            hasData = !imgRGBBuf.empty() && !mpImuGb->imuBuf.empty();
        }

        if (!hasData)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            // Variables
            double tIm = 0;
            cv::Mat im, depth;
            Eigen::Vector3f Wbb;
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr msgPC;

            // Get the first image from the buffer
            tIm = rclcpp::Time(imgRGBBuf.front()->header.stamp).seconds();
            double latestImuTime = rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp).seconds();

            if (tIm > latestImuTime)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[Warning] Skipping frame ... RGB time (%.3f) is ahead of latest IMU time (%.3f)!",
                            tIm, latestImuTime);
                continue;
            }

            // Get the RGB image, depth image, and pointcloud from the buffers
            this->mBufMutex.lock();
            rclcpp::Time msg_time = imgRGBBuf.front()->header.stamp;
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            msgPC = imgPCBuf.front();
            imgPCBuf.pop();
            this->mBufMutex.unlock();

            vImuMeas.clear();
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                while (!mpImuGb->imuBuf.empty() && rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds() <= tIm)
                {
                    double t = rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Convert pointclouds from ros to pcl format
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*msgPC, *cloud);

            // Find the marker with the minimum time difference compared to the current frame
            std::pair<double, std::vector<ORB_SLAM3::Marker *>> result = findNearestMarker(tIm);
            double minMarkerTimeDiff = result.first;
            std::vector<ORB_SLAM3::Marker *> matchedMarkers = result.second;

            // Tracking process sends markers found in this frame for tracking and clears the buffer
            if (minMarkerTimeDiff < 0.05)
            {
                pSLAM->TrackRGBD(im, depth, cloud, tIm, vImuMeas, "", matchedMarkers);
                markersBuffer.clear();
            }
            else
                pSLAM->TrackRGBD(im, depth, cloud, tIm, vImuMeas);

            publishTopics(msg_time, Wbb);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");

    std::string nodeName = node->get_name();

    // Parameters
    node->declare_parameter<double>("yaw", 0.0);
    node->declare_parameter<double>("roll", 0.0);
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<bool>("enable_pangolin", true);
    node->declare_parameter<bool>("static_transform", false);
    node->declare_parameter<std::string>("frame_imu", "imu");
    node->declare_parameter<std::string>("frame_map", "map");
    node->declare_parameter<bool>("colored_pointcloud", true);
    node->declare_parameter<bool>("publish_pointclouds", true);
    node->declare_parameter<std::string>("frame_world", "world");
    node->declare_parameter<std::string>("frame_camera", "camera");
    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->declare_parameter<std::string>("sys_params_file", "file_not_set");
    node->declare_parameter<std::string>("frame_structural_element", "struc_elem");
    node->declare_parameter<std::string>("frame_building_component", "build_comp");

    std::string vocFile = node->get_parameter("voc_file").as_string();
    std::string settingsFile = node->get_parameter("settings_file").as_string();
    std::string sysParamsFile = node->get_parameter("sys_params_file").as_string();

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] 'vocabulary' and 'settings' are not provided in the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] The `YAML` file containing system parameters is not provided in the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    yaw = node->get_parameter("yaw").as_double();
    roll = node->get_parameter("roll").as_double();
    pitch = node->get_parameter("pitch").as_double();
    frameImu = node->get_parameter("frame_imu").as_string();
    frameMap = node->get_parameter("frame_map").as_string();
    frameWorld = node->get_parameter("frame_world").as_string();
    frameCamera = node->get_parameter("frame_camera").as_string();
    colorPointcloud = node->get_parameter("colored_pointcloud").as_bool();
    pubPointClouds = node->get_parameter("publish_pointclouds").as_bool();
    frameBC = node->get_parameter("frame_building_component").as_string();
    frameSE = node->get_parameter("frame_structural_element").as_string();
    pubStaticTransform = node->get_parameter("static_transform").as_bool();
    bool enablePangolin = node->get_parameter("enable_pangolin").as_bool();

    // Initializing system threads and getting ready to process frames
    auto imugb = std::make_shared<ImuGrabber>();
    auto igb = std::make_shared<ImageGrabber>(imugb);

    sensorType = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(vocFile, settingsFile, sysParamsFile, sensorType, enablePangolin);

    // Subscribe to get raw images (message_filters in ROS2)
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;
    using sensor_msgs::msg::PointCloud2;

    // Subscriber to IMU data with reliable QoS
    rclcpp::QoS imu_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    auto subImu = node->create_subscription<Imu>(
        "/imu", imu_qos,
        [imugb, node](const Imu::ConstSharedPtr msg)
        { imugb->GrabImu(msg); });

    auto subImgRGB = std::make_shared<Subscriber<Image>>(node.get(), "/camera/rgb/image_raw");
    auto subPointcloud = std::make_shared<Subscriber<PointCloud2>>(node.get(), "/camera/depth/points");
    auto subImgDepth = std::make_shared<Subscriber<Image>>(node.get(), "/camera/depth_registered/image_raw");

    typedef ApproximateTime<Image, Image, PointCloud2> syncPolicy;
    auto sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10), *subImgRGB, *subImgDepth, *subPointcloud);
    sync->registerCallback(
        std::bind(&ImageGrabber::GrabRGBD, igb.get(),
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Subscriber to get segmentation results from the SemanticSegmenter module
    auto subSegmentedImage = node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
        "/camera/color/image_segment", 50,
        [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
        { igb->GrabSegmentation(*msg); });

    // Subsriber to get skeletonized graph from the `voxblox` module
    auto subVoxbloxSkeletonMesh = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/voxblox_skeletonizer/sparse_graph", 1,
        [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        { igb->GrabVoxbloxSkeletonGraph(*msg); });

    static std::shared_ptr<image_transport::ImageTransport> image_transport = std::make_shared<image_transport::ImageTransport>(node);
    setupPublishers(node, image_transport, nodeName);

    // Syncing images with IMU
    std::thread sync_thread(&ImageGrabber::SyncWithImu, igb);

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();

    // Signal the sync thread to stop and wait for it
    igb->mustStop = true;
    sync_thread.join();

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                            const sensor_msgs::msg::Image::ConstSharedPtr &msgD,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC)
{
    mBufMutex.lock();

    if (!imgRGBBuf.empty())
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);

    if (!imgPCBuf.empty())
        imgPCBuf.pop();
    imgPCBuf.push(msgPC);

    mBufMutex.unlock();
}

/**
 * @brief Callback function to get the markers detected by the `aruco_ros` library
 *
 * @param msgMarkerArray The markers detected by the `aruco_ros` library
 */
// void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &msgMarkerArray)
// {
//     // Pass the visited markers to a buffer to be processed later
//     addMarkersToBuffer(msgMarkerArray);
// }

/**
 * @brief Callback function to get the skeleton graph from the `voxblox` module
 *
 * @param msgSkeletonGraphs The skeleton graph from the `voxblox` module
 */
void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}