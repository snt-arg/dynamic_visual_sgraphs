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

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber() : rclcpp::Node("image_grabber") {}

    // void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

    std::mutex mBufMutex;
    // queue<sensor_msgs::ImuConstPtr> imuBuf;
    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {}

    void SyncWithImu();

    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    // cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    // void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsgg &msgSegImage);
    // void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);
    // void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
    //               const sensor_msgs::PointCloud2ConstPtr &msgPC);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);
    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msgD,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC);

    ImuGrabber *mpImuGb;
    std::mutex mBufMutex;
    // queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    // queue<sensor_msgs::PointCloud2ConstPtr> imgPCBuf;
    queue<sensor_msgs::msg::Image::ConstSharedPtr> imgRGBBuf, imgDBuf;
    queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> imgPCBuf;
};

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "RGBD_Inertial");
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

//     if (argc > 1)
//     {
//         ROS_WARN("Arguments supplied via command line are ignored.");
//     }

//     std::string node_name = ros::this_node::getName();

//     ros::NodeHandle nodeHandler;
//     image_transport::ImageTransport image_transport(nodeHandler);

//     std::string voc_file, settings_file, sys_params_file;
//     nodeHandler.param<std::string>(node_name + "/sys_params_file", sys_params_file, "file_not_set");
//     nodeHandler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
//     nodeHandler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

//     if (voc_file == "file_not_set" || settings_file == "file_not_set")
//     {
//         ROS_ERROR("Please provide voc_file and settings_file in the launch file");
//         ros::shutdown();
//         return 1;
//     }

//     if (sys_params_file == "file_not_set")
//     {
//         ROS_ERROR("Please provide the YAML file containing system parameters in the launch file!");
//         ros::shutdown();
//         return 1;
//     }

//     bool enable_pangolin;
//     nodeHandler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

//     nodeHandler.param<double>(node_name + "/yaw", yaw, 0.0);
//     nodeHandler.param<double>(node_name + "/roll", roll, 0.0);
//     nodeHandler.param<double>(node_name + "/pitch", pitch, 0.0);

//     nodeHandler.param<std::string>(node_name + "/frame_map", frameMap, "map");
//     nodeHandler.param<std::string>(node_name + "/frame_imu", frameImu, "imu");
//     nodeHandler.param<std::string>(node_name + "/frame_camera", frameCamera, "camera");
//     nodeHandler.param<std::string>(node_name + "/frame_world", frameWorld, "world");
//     nodeHandler.param<bool>(node_name + "/static_transform", pubStaticTransform, false);
//     nodeHandler.param<std::string>(node_name + "/frame_building_component", frameBC, "build_comp");
//     nodeHandler.param<std::string>(node_name + "/frame_structural_element", frameSE, "struc_elem");

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ImuGrabber imugb;
//     ImageGrabber igb(&imugb);
//     sensorType = ORB_SLAM3::System::IMU_RGBD;

//     pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sys_params_file, sensorType, enable_pangolin);

//     // Subscribe to get raw images and IMU data
//     ros::Subscriber sub_imu = nodeHandler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
//     message_filters::Subscriber<sensor_msgs::Image> subImgRGB(nodeHandler, "/camera/rgb/image_raw", 500);
//     message_filters::Subscriber<sensor_msgs::Image> subImgDepth(nodeHandler, "/camera/depth_registered/image_raw", 500);

//     // Subscribe to get pointcloud from depth sensor
//     message_filters::Subscriber<sensor_msgs::PointCloud2> subPointcloud(nodeHandler, "/camera/pointcloud", 500);

//     // Synchronization of raw and depth images
//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2>
//         syncPolicy;
//     message_filters::Synchronizer<syncPolicy> sync(syncPolicy(500), subImgRGB, subImgDepth, subPointcloud);
//     sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2, _3));

//     // Subscribe to the markers detected by `aruco_ros` library
//     // ros::Subscriber sub_aruco = nodeHandler.subscribe("/aruco_marker_publisher/markers", 1,
//     //                                                   &ImageGrabber::GrabArUcoMarker, &igb);

//     // Subscriber for images obtained from the Semantic Segmentater
//     ros::Subscriber sub_segmented_img = nodeHandler.subscribe("/camera/color/image_segment", 50,
//                                                               &ImageGrabber::GrabSegmentation, &igb);

//     // Subscriber to get the mesh from voxblox
//     ros::Subscriber voxblox_skeleton_mesh = nodeHandler.subscribe("/voxblox_skeletonizer/sparse_graph", 1,
//                                                                   &ImageGrabber::GrabVoxbloxSkeletonGraph, &igb);

//     setupPublishers(nodeHandler, image_transport, node_name);
//     setupServices(nodeHandler, node_name);

//     // Syncing images with IMU
//     std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

//     ros::spin();

//     // Stop all threads
//     pSLAM->Shutdown();
//     ros::shutdown();

//     return 0;
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("RGBD_Inertial");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");

    std::string node_name = node->get_name();

    // Declare and get parameters
    node->declare_parameter<std::string>("sys_params_file", "file_not_set");
    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->declare_parameter<bool>("enable_pangolin", true);
    node->declare_parameter<double>("yaw", 0.0);
    node->declare_parameter<double>("roll", 0.0);
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<std::string>("frame_map", "map");
    node->declare_parameter<std::string>("frame_imu", "imu");
    node->declare_parameter<std::string>("frame_camera", "camera");
    node->declare_parameter<std::string>("frame_world", "world");
    node->declare_parameter<bool>("static_transform", false);
    node->declare_parameter<std::string>("frame_building_component", "build_comp");
    node->declare_parameter<std::string>("frame_structural_element", "struc_elem");

    std::string sys_params_file = node->get_parameter("sys_params_file").as_string();
    std::string voc_file = node->get_parameter("voc_file").as_string();
    std::string settings_file = node->get_parameter("settings_file").as_string();

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file in the launch file");
        rclcpp::shutdown();
        return 1;
    }
    if (sys_params_file == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "Please provide the YAML file containing system parameters in the launch file!");
        rclcpp::shutdown();
        return 1;
    }

    bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();
    yaw = node->get_parameter("yaw").as_double();
    roll = node->get_parameter("roll").as_double();
    pitch = node->get_parameter("pitch").as_double();
    frameMap = node->get_parameter("frame_map").as_string();
    frameImu = node->get_parameter("frame_imu").as_string();
    frameCamera = node->get_parameter("frame_camera").as_string();
    frameWorld = node->get_parameter("frame_world").as_string();
    pubStaticTransform = node->get_parameter("static_transform").as_bool();
    frameBC = node->get_parameter("frame_building_component").as_string();
    frameSE = node->get_parameter("frame_structural_element").as_string();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImuGrabber imugb;
    ImageGrabber igb(&imugb);
    sensorType = ORB_SLAM3::System::IMU_RGBD;

    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sys_params_file, sensorType, enable_pangolin);

    // --- ROS2 Subscribers ---
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;
    using sensor_msgs::msg::PointCloud2;

    // IMU
    auto sub_imu = node->create_subscription<Imu>(
        "/imu", 1000,
        [&imugb](const Imu::SharedPtr msg)
        { imugb.GrabImu(msg); });

    // Images and pointcloud (message_filters)
    auto subImgRGB = std::make_shared<Subscriber<Image>>(node.get(), "/camera/rgb/image_raw");
    auto subImgDepth = std::make_shared<Subscriber<Image>>(node.get(), "/camera/depth_registered/image_raw");
    auto subPointcloud = std::make_shared<Subscriber<PointCloud2>>(node.get(), "/camera/pointcloud");

    typedef ApproximateTime<Image, Image, PointCloud2> syncPolicy;
    auto sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10), *subImgRGB, *subImgDepth, *subPointcloud);
    sync->registerCallback(
        [&igb](const Image::ConstSharedPtr msgRGB, const Image::ConstSharedPtr msgD, const PointCloud2::ConstSharedPtr msgPC)
        {
            igb.GrabRGBD(msgRGB, msgD, msgPC);
        });

    // Segmentation and Voxblox
    auto sub_segmented_img = node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
        "/camera/color/image_segment", 50,
        [&igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
        { igb.GrabSegmentation(*msg); });

    auto voxblox_skeleton_mesh = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/voxblox_skeletonizer/sparse_graph", 1,
        [&igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        { igb.GrabVoxbloxSkeletonGraph(*msg); });

    // TODO: setupPublishers and setupServices for ROS2 if needed

    // Syncing images with IMU
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}

// void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
//                             const sensor_msgs::PointCloud2ConstPtr &msgPC)
void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
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
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while (1)
    {
        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im, depth;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr msgPC;
            double tIm = 0;

            tIm = imgRGBBuf.front()->header.stamprclcpp::Time::seconds();
            if (tIm > mpImuGb->imuBuf.back()->header.stamprclcpp::Time::seconds())
                continue;

            this->mBufMutex.lock();
            rclcpp::Time msg_time = imgRGBBuf.front()->header.stamp;
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            msgPC = imgPCBuf.front();
            imgPCBuf.pop();

            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            vImuMeas.clear();
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamprclcpp::Time::seconds() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamprclcpp::Time::seconds();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Pointcloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Convert pointclouds from ros to pcl format
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

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}

// void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &markerArray)
// {
//     // Pass the visited markers to a buffer to be processed later
//     // addMarkersToBuffer(markerArray);
// }

void ImageGrabber::GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsgg &msgSegImage)
{
    // Fetch the segmentation results
    cv_bridge::CvImageConstPtr cv_imgSeg;
    uint64_t key_frame_id = msgSegImage.key_frame_id.data;

    try
    {
        cv_imgSeg = cv_bridge::toCvCopy(msgSegImage.segmentedImage, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // convert to PCL PointCloud2 from sensor_msgs PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmentedImageProbability, *pclPc2SegPrb);

    // Create the tuple to be appended to the segmentedImageBuffer
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(key_frame_id, cv_imgSeg->image, pclPc2SegPrb);

    // Add the segmented image to a buffer to be processed in the SemanticSegmentation thread
    pSLAM->addSegmentedImage(&tuple);
}

void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}