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
    ImageGrabber() : rclcpp::Node("grabber")
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage);
    void GrabGNNRoomCandidates(const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);
    void GrabGNNRoomCandidates(const situational_graphs_msgs::msg::RoomsData::SharedPtr &msgGNNRooms);
    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr &msgD,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC);
};

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
    auto igb = std::make_shared<ImageGrabber>();

    sensorType = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(vocFile, settingsFile, sysParamsFile, sensorType, enablePangolin);

    // Subscribe to get raw images (message_filters in ROS2)
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::PointCloud2;

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

    // Subscriber to get room candidates from the GNN module (legacy)
    auto subGNNRooms_legacy = node->create_subscription<situational_graphs_msgs::msg::RoomsData>(
        "/room_segmentation/room_data", 10,
        [igb](const situational_graphs_msgs::msg::RoomsData::SharedPtr msg)
        { igb->GrabGNNRoomCandidates(msg); });

    // Subscriber to get room candidates from the GNN module (new version)
    auto subGNNRooms_new = node->create_subscription<vs_graphs::msg::VSGraphsAllDetectdetRooms>(
        "/gnn_room_detector", 1,
        [igb](const vs_graphs::msg::VSGraphsAllDetectdetRooms::SharedPtr msg)
        { igb->GrabGNNRoomCandidates(*msg); });

    static std::shared_ptr<image_transport::ImageTransport> image_transport = std::make_shared<image_transport::ImageTransport>(node);
    setupPublishers(node, image_transport, nodeName);

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}

/**
 * @brief Callback function to grab RGB-D images and point clouds
 *
 * @param msgRGB The RGB image message
 * @param msgD The depth image message
 * @param msgPC The point cloud message
 */
void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                            const sensor_msgs::msg::Image::ConstSharedPtr &msgD,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC)
{
    // Variables
    cv_bridge::CvImageConstPtr cv_ptrD, cv_ptrRGB;

    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] Problem occured while running `cv_bridge`: %s", e.what());
        return;
    }

    // Convert pointclouds from ros to pcl format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msgPC, *cloud);

    // Find the marker with the minimum time difference compared to the current frame
    std::pair<double, std::vector<ORB_SLAM3::Marker *>>
        foundMarkerRes = findNearestMarker(cv_ptrRGB->header.stamp.sec + cv_ptrRGB->header.stamp.nanosec * 1e-9);
    double minMarkerTimeDiff = foundMarkerRes.first;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers = foundMarkerRes.second;

    // Tracking process sends markers found in this frame for tracking and clears the buffer
    if (minMarkerTimeDiff < 0.05)
    {
        pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                         cv_ptrRGB->header.stamp.sec + cv_ptrRGB->header.stamp.nanosec * 1e-9,
                         {}, "", matchedMarkers);
        markersBuffer.clear();
    }
    else
    {
        pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                         cv_ptrRGB->header.stamp.sec + cv_ptrRGB->header.stamp.nanosec * 1e-9);
    }

    rclcpp::Time msgTime = cv_ptrRGB->header.stamp;
    publishTopics(msgTime, Eigen::Vector3f::Zero(), msgPC);
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

/**
 * @brief Callback function to get the room candidates detected by the GNN room detector module (legacy version)
 *
 * @param msgGNNRooms The room candidates detected by the GNN module
 */
void ImageGrabber::GrabGNNRoomCandidates(const situational_graphs_msgs::msg::RoomsData::SharedPtr &msgGNNRooms)
{
    // Set the GNN room candidates in the SLAM system
    // setGNNBasedRoomCandidates(msgGNNRooms);
}

/**
 * @brief Callback function to get the room candidates detected by the GNN room detector module (new version)
 *
 * @param msgGNNRooms The room candidates detected by the GNN module
 */
void ImageGrabber::GrabGNNRoomCandidates(const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms)
{
    // Set the GNN room candidates in the SLAM system
    setGNNBasedRoomCandidates(msgGNNRooms);
}