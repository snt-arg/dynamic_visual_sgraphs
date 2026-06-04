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

#include <atomic>
#include <limits>

using namespace std;

class ImuGrabber : public rclcpp::Node
{
public:
    ImuGrabber() : rclcpp::Node("imu_grabber", rclcpp::NodeOptions().use_global_arguments(false))
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    std::mutex mBufMutex;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(std::shared_ptr<ImuGrabber> imuGrabber)
        : rclcpp::Node("image_grabber", rclcpp::NodeOptions().use_global_arguments(false)),
          mpImuGb(std::move(imuGrabber))
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void SyncWithImu();

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);

    std::mutex mBufMutex;
    std::atomic<bool> mustStop{false};
    std::shared_ptr<ImuGrabber> mpImuGb;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img0Buf;

private:
    double minMarkerTimeDiff = std::numeric_limits<double>::max();
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] `cv_bridge` exception: %s", e.what());
        return cv::Mat();
    }

    std::pair<double, std::vector<ORB_SLAM3::Marker *>> result =
        findNearestMarker(rclcpp::Time(cv_ptr->header.stamp).seconds());
    minMarkerTimeDiff = result.first;
    matchedMarkers = result.second;

    return cv_ptr->image.clone();
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
        bool hasData = false;
        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            std::lock_guard<std::mutex> lock2(mpImuGb->mBufMutex);
            hasData = !img0Buf.empty() && !mpImuGb->imuBuf.empty();
        }

        if (!hasData)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat im;
        double tIm = 0.0;
        rclcpp::Time msgTime;
        Eigen::Vector3f Wbb = Eigen::Vector3f::Zero();
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            std::lock_guard<std::mutex> lock2(mpImuGb->mBufMutex);

            if (img0Buf.empty() || mpImuGb->imuBuf.empty())
                continue;

            tIm = rclcpp::Time(img0Buf.front()->header.stamp).seconds();
            const double latestImuTime = rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp).seconds();

            if (tIm > latestImuTime)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[Warning] Waiting for IMU data ... image time (%.3f) is ahead of latest IMU time (%.3f)!",
                                     tIm, latestImuTime);
                continue;
            }

            msgTime = rclcpp::Time(img0Buf.front()->header.stamp);
            im = GetImage(img0Buf.front());
            img0Buf.pop();

            while (!mpImuGb->imuBuf.empty() &&
                   rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds() <= tIm)
            {
                const auto imuMsg = mpImuGb->imuBuf.front();
                const double t = rclcpp::Time(imuMsg->header.stamp).seconds();
                const cv::Point3f acc(imuMsg->linear_acceleration.x,
                                      imuMsg->linear_acceleration.y,
                                      imuMsg->linear_acceleration.z);
                const cv::Point3f gyr(imuMsg->angular_velocity.x,
                                      imuMsg->angular_velocity.y,
                                      imuMsg->angular_velocity.z);

                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                Wbb << imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z;
                mpImuGb->imuBuf.pop();
            }
        }

        if (im.empty())
            continue;

        if (minMarkerTimeDiff < 0.05)
        {
            pSLAM->TrackMonocular(im, tIm, vImuMeas, "", matchedMarkers);
            markersBuffer.clear();
        }
        else
            pSLAM->TrackMonocular(im, tIm, vImuMeas);

        publishTopics(msgTime, Wbb);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/**
 * @brief Callback function to get scene segmentation results from the SemanticSegmenter module
 *
 * @param msgSegImage The segmentation results from the SemanticSegmenter
 */
void ImageGrabber::GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage)
{
    cv_bridge::CvImageConstPtr cvImgSeg;
    uint64_t keyFrameId = msgSegImage.key_frame_id.data;

    try
    {
        cvImgSeg = cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(msgSegImage.segmented_image),
                                       sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] `cv_bridge` exception: %s", e.what());
        return;
    }

    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmented_image_probability, *pclPc2SegPrb);

    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(keyFrameId, cvImgSeg->image, pclPc2SegPrb);
    pSLAM->addSegmentedImage(&tuple);
}

/**
 * @brief Callback function to get the skeleton graph from the `voxblox` module
 *
 * @param msgSkeletonGraphs The skeleton graph from the `voxblox` module
 */
void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");

    std::string nodeName = node->get_name();

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
        RCLCPP_ERROR(node->get_logger(), "[Error] 'voc_file' and 'settings_file' are not provided in the launch file! Exiting...");
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

    auto imugb = std::make_shared<ImuGrabber>();
    auto igb = std::make_shared<ImageGrabber>(imugb);

    sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(vocFile, settingsFile, sysParamsFile, sensorType, enablePangolin);

    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;

    rclcpp::QoS sensorQos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensorQos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    sensorQos.durability(rclcpp::DurabilityPolicy::Volatile);

    auto subImu = node->create_subscription<Imu>(
        "/imu", sensorQos,
        [imugb](const Imu::ConstSharedPtr msg)
        { imugb->GrabImu(msg); });

    auto subImg = node->create_subscription<Image>(
        "/camera/image_raw", sensorQos,
        [igb](const Image::ConstSharedPtr msg)
        { igb->GrabImage(msg); });

    // Subscribe to the markers detected by `aruco_ros` library
    // auto subAruco = node->create_subscription<aruco_msgs::msg::MarkerArray>(
    //     "/aruco_marker_publisher/markers", 1,
    //     [igb](const aruco_msgs::msg::MarkerArray::SharedPtr msg)
    //     { igb->GrabArUcoMarker(*msg); });

    auto subSegmentedImage = node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
        "/camera/color/image_segment", 50,
        [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
        { igb->GrabSegmentation(*msg); });

    auto subVoxbloxSkeletonMesh = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/voxblox_skeletonizer/sparse_graph", 1,
        [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        { igb->GrabVoxbloxSkeletonGraph(*msg); });

    static std::shared_ptr<image_transport::ImageTransport> imageTransport =
        std::make_shared<image_transport::ImageTransport>(node);
    setupPublishers(node, imageTransport, nodeName);
    setupServices(node, nodeName);

    std::thread syncThread(&ImageGrabber::SyncWithImu, igb);

    rclcpp::spin(node);

    pSLAM->Shutdown();
    igb->mustStop = true;
    syncThread.join();

    rclcpp::shutdown();

    return 0;
}
