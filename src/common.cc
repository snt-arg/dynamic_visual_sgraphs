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
#include <rclcpp/rclcpp.hpp>

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
bool colorPointcloud = true;
double roll = 0, pitch = 0, yaw = 0;
bool pubStaticTransform, pubPointClouds;
std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;
std::shared_ptr<image_transport::Publisher> pubTrackingImage;
// std::shared_ptr<tf2::TransformListener> transformListener;
std::shared_ptr<rviz_visual_tools::RvizVisualTools> visualTools;
std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
std::string frameWorld, frameCamera, frameImu, frameMap, frameBC, frameSE;

rclcpp::Time lastPlanePublishTime(0, 0, RCL_ROS_TIME);
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubKeyFrameList;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDoor;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllMappoints;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose;
rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBuildingComponents;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrackedMappoints;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreespaceCluster;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPlaneLabel;
rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr pubAllWalls_new;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedPointcloud;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCameraPoseVis;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKeyFrameMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubFiducialMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubStructuralElements;
rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr pubAllWalls_legacy;

void saveMapService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMap(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), "Map was saved as %s.osa", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Map could not be saved.");
}

void saveMapPointsAsPCDService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMapPointsAsPCD(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), "Map points were saved as %s.pcd", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Map points could not be saved.");
}

void saveTrajectoryService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    const std::string cameraTrajectoryFile = req->name + "_cam_traj.txt";
    const std::string keyframeTrajectoryFile = req->name + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cameraTrajectoryFile);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(keyframeTrajectoryFile);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        res->success = false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
        res->success = false;
    }

    if (!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Estimated trajectory could not be saved.");
}

void setupServices(std::shared_ptr<rclcpp::Node> node, const std::string &node_name)
{
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_map", &saveMapService);
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_map_points", &saveMapPointsAsPCDService);
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_traj", &saveTrajectoryService);
}

void setupPublishers(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<image_transport::ImageTransport> image_transport, const std::string &node_name)
{
    // Basic
    pubKeyFrameList = node->create_publisher<nav_msgs::msg::Path>(node_name + "/keyframe_list", 2);
    pubAllMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);
    pubCameraPose = node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);
    pubKFImage = node->create_publisher<segmenter_ros::msg::VSGraphDataMsg>(node_name + "/keyframe_image", 50);
    pubTrackedMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);
    pubKeyFrameMarker = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/kf_markers", 1);
    pubFreespaceCluster = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/freespace_clusters", 1);
    pubCameraPoseVis = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/camera_pose_vis", 1);
    pubTrackingImage = std::make_shared<image_transport::Publisher>(image_transport->advertise(node_name + "/tracking_image", 1));

    // Entities
    pubDoor = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/doors", 1);
    pubFiducialMarker = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/fiducial_markers", 1);

    // Building Components
    pubPlaneLabel = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/plane_labels", 1);
    pubBuildingComponents = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/building_components", 1);
    pubSegmentedPointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/segmented_point_clouds", 1);

    // All Walls
    pubAllWalls_new = node->create_publisher<vs_graphs::msg::VSGraphsAllWallsData>(node_name + "/all_mapped_walls", 1);
    pubAllWalls_legacy = node->create_publisher<situational_graphs_msgs::msg::PlanesData>(node_name + "/all_mapped_walls", 1);

    // Structural Elements
    pubStructuralElements = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/structural_elements", 1);

    // Get body odometry if IMU data is also available
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
        pubOdometry = node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);

    // Visualizing Planes
    visualTools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
        frameBC, "/plane_visuals", node);
    visualTools->setAlpha(0.5);
    visualTools->loadMarkerPub();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void publishTopics(rclcpp::Time msgTime, Eigen::Vector3f Wbb)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publishCameraPose(Twc, msgTime);
    publishTFTransform(Twc, frameWorld, frameCamera, msgTime);

    // Set a static transform between the world and map frame
    if (pubStaticTransform)
        publishStaticTFTransform(frameWorld, frameMap, msgTime);

    // Get KeyFrames
    std::vector<ORB_SLAM3::KeyFrame *> keyframes = pSLAM->GetAllKeyFrames();

    // Setup publishers
    publishKeyFrameImages(keyframes, msgTime);
    publishKeyFrameMarkers(keyframes, msgTime);
    publishDoors(pSLAM->GetAllDoors(), msgTime);
    publishFiducialMarkers(pSLAM->GetAllMarkers(), msgTime);
    publishTrackingImage(pSLAM->GetCurrentFrame(), msgTime);
    publishStructuralElements(pSLAM->GetAllRooms(), pSLAM->GetAllFloors(), msgTime);

    // Publish all mapped walls for GNN-based room detection
    publishAllMappedWalls(pSLAM->GetAllPlanes(), msgTime);

    // Publish pointclouds
    if (pubPointClouds)
    {
        publishSegmentedCloud(keyframes, msgTime);
        publishPlanes(pSLAM->GetAllPlanes(), msgTime);
        publishAllPoints(pSLAM->GetAllMapPoints(), msgTime);
        publishTrackedPoints(pSLAM->GetTrackedMapPoints(), msgTime);
        publishFreeSpaceClusters(pSLAM->getSkeletonCluster(), msgTime);
    }
    else
        clearKFClsClouds(keyframes);

    // IMU-specific topics
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f Wwb = Rwb * Wbb;

        publishTFTransform(Twb, frameWorld, frameImu, msgTime);
        publishBodyOdometry(Twb, Vwb, Wwb, msgTime);
    }
}

void publishBodyOdometry(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msgTime)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.child_frame_id = frameImu;
    odom_msg.header.frame_id = frameWorld;
    odom_msg.header.stamp = msgTime;

    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
    odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
    odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
    odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    pubOdometry->publish(odom_msg);
}

void publishCameraPose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msgTime)
{
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.frame_id = frameCamera;
    poseMsg.header.stamp = msgTime;

    poseMsg.pose.position.x = Tcw_SE3f.translation().x();
    poseMsg.pose.position.y = Tcw_SE3f.translation().y();
    poseMsg.pose.position.z = Tcw_SE3f.translation().z();

    poseMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    poseMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    poseMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    poseMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pubCameraPose->publish(poseMsg);

    // Add a marker for visualization
    visualization_msgs::msg::Marker cameraVisual;
    visualization_msgs::msg::MarkerArray cameraVisualList;

    cameraVisual.id = 1;
    cameraVisual.color.a = 0.7;
    cameraVisual.scale.x = 0.5;
    cameraVisual.scale.y = 0.5;
    cameraVisual.scale.z = 0.5;
    cameraVisual.ns = "camera_pose";
    cameraVisual.header.stamp = msgTime;
    cameraVisual.action = cameraVisual.ADD;
    cameraVisual.header.frame_id = frameWorld;
    cameraVisual.mesh_use_embedded_materials = true;
    cameraVisual.lifetime = rclcpp::Duration::from_seconds(0);
    cameraVisual.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    cameraVisual.mesh_resource =
        "package://vs_graphs/config/Assets/camera.dae";

    cameraVisual.pose.position.x = Tcw_SE3f.translation().x();
    cameraVisual.pose.position.y = Tcw_SE3f.translation().y();
    cameraVisual.pose.position.z = Tcw_SE3f.translation().z();
    cameraVisual.pose.orientation.x = Tcw_SE3f.unit_quaternion().x();
    cameraVisual.pose.orientation.y = Tcw_SE3f.unit_quaternion().y();
    cameraVisual.pose.orientation.z = Tcw_SE3f.unit_quaternion().z();
    cameraVisual.pose.orientation.w = Tcw_SE3f.unit_quaternion().w();

    cameraVisualList.markers.push_back(cameraVisual);

    pubCameraPoseVis->publish(cameraVisualList);
}

void publishTFTransform(Sophus::SE3f T_SE3f, std::string parentFrameId, std::string childFrameId, rclcpp::Time msgTime)
{
    // Variables
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = msgTime;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the values of transform messages
    transformStamped.transform.translation.x = T_SE3f.translation().x();
    transformStamped.transform.translation.y = T_SE3f.translation().y();
    transformStamped.transform.translation.z = T_SE3f.translation().z();

    // Fill rotation
    transformStamped.transform.rotation.x = T_SE3f.unit_quaternion().x();
    transformStamped.transform.rotation.y = T_SE3f.unit_quaternion().y();
    transformStamped.transform.rotation.z = T_SE3f.unit_quaternion().z();
    transformStamped.transform.rotation.w = T_SE3f.unit_quaternion().w();

    if (tfBroadcaster)
        tfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "TF broadcaster is not initialized.");
}

void publishStaticTFTransform(std::string parentFrameId, std::string childFrameId, rclcpp::Time msgTime)
{
    // Variables
    tf2::Quaternion quat;
    geometry_msgs::msg::TransformStamped transformStamped;

    // Set the values of transform messages
    transformStamped.header.stamp = msgTime;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the translation to zero (static transform)
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    // Set the rotation using roll, pitch, yaw
    quat.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    if (staticTfBroadcaster)
        staticTfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Static TF broadcaster is not initialized.");
}

void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>> clusterPoints, rclcpp::Time msgTime)
{
    // Check if the cluster points are empty
    if (clusterPoints.empty())
        return;

    // Variables
    int colorIndex = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr freeSpaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<std::vector<uint8_t>> fixedColors = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}, {128, 0, 0}};

    // Loop through all the cluster points and add them to the point cloud
    for (const auto &cluster : clusterPoints)
    {
        // Variables
        std::vector<uint8_t> color = fixedColors[colorIndex];
        // Loop through all the points in the current cluster
        for (const auto &point : cluster)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x();
            newPoint.y = point.y();
            newPoint.z = point.z();
            newPoint.r = color[0];
            newPoint.g = color[1];
            newPoint.b = color[2];
            freeSpaceCloud->push_back(newPoint);
        }
        // Increment the color index
        colorIndex += 1;
        // if (colorIndex == fixedColors.size())
        if (colorIndex == static_cast<int>(fixedColors.size()))
            colorIndex = 0;
    }

    // Check if the point cloud is empty
    if (freeSpaceCloud->empty())
        return;

    // Convert the point cloud to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*freeSpaceCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = frameWorld;

    // Publish the point cloud
    pubFreespaceCluster->publish(cloudMsg);
}

void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, rclcpp::Time msgTime)
{
    // Check all keyframes and publish the ones that have not been published for Semantic Segmentation yet
    for (auto &keyframe : keyframe_vec)
    {
        if (keyframe->isPublished)
            continue;

        // Create an object of VSGraphDataMsg
        segmenter_ros::msg::VSGraphDataMsg vsGraphPublisher = segmenter_ros::msg::VSGraphDataMsg();
        std_msgs::msg::Header header;
        header.stamp = msgTime;
        header.frame_id = frameWorld;
        std_msgs::msg::UInt64 kfId;
        kfId.data = keyframe->mnId;
        const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
            cv_bridge::CvImage(header, "bgr8", keyframe->mImage).toImageMsg();

        vsGraphPublisher.header = header;
        vsGraphPublisher.key_frame_id = kfId;
        vsGraphPublisher.key_frame_image = *rendered_image_msg;

        pubKFImage->publish(vsGraphPublisher);
        keyframe->isPublished = true;
    }
}

void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> walls, rclcpp::Time msgTime)
{
    // Check the proper version of the GNN-based room detection
    bool isLegacy = ORB_SLAM3::SystemParams::GetParams()->room_seg.gnn_version == 1;
    std::cout << "GNN version: " << isLegacy << std::endl;

    if (isLegacy)
    {
        // Variables
        situational_graphs_msgs::msg::PlanesData wallDataMsg;

        // Fill the data message with wall information
        wallDataMsg.header.stamp = msgTime;
        wallDataMsg.header.frame_id = frameWorld;

        // Fill in the walls data
        for (const auto &wall : walls)
        {
            if (!wall || wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
                continue;

            // Fill the wall data
            situational_graphs_msgs::msg::PlaneData wallData;

            // wallData.length = length;
            wallData.id = wall->getId();
            wallData.d = wall->getGlobalEquation().d();
            wallData.nx = wall->getGlobalEquation().normal().x();
            wallData.ny = wall->getGlobalEquation().normal().y();
            wallData.nz = wall->getGlobalEquation().normal().z();

            // Add the wall to the message
            wallDataMsg.walls.push_back(wallData);
        }

        // Publish all mapped walls
        pubAllWalls_legacy->publish(wallDataMsg);
    }
    else
    {
        // Variables
        vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

        // Fill the data message with wall information
        wallDataMsg.header.stamp = msgTime;
        wallDataMsg.header.frame_id = frameWorld;

        // Fill in the walls data
        for (const auto &wall : walls)
        {
            if (!wall || wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
                continue;

            // Calculate the length of the wall
            float length = 0.0f;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud = wall->getMapClouds();
            if (wallCloud && wallCloud->points.size() > 1)
            {
                // Calculate the length of the wall by finding the distance between the first and last points
                Eigen::Vector3f startPoint(wallCloud->points.front().x, wallCloud->points.front().y, wallCloud->points.front().z);
                Eigen::Vector3f endPoint(wallCloud->points.back().x, wallCloud->points.back().y, wallCloud->points.back().z);
                length = (endPoint - startPoint).norm();
            }

            // Fill the wall data
            vs_graphs::msg::VSGraphsWallData wallData;

            wallData.length = length;
            wallData.id = wall->getId();
            wallData.centroid.x = wall->getCentroid().x();
            wallData.centroid.y = wall->getCentroid().y();
            wallData.centroid.z = wall->getCentroid().z();
            wallData.normal.x = wall->getGlobalEquation().normal().x();
            wallData.normal.y = wall->getGlobalEquation().normal().y();
            wallData.normal.z = wall->getGlobalEquation().normal().z();

            // Add the wall to the message
            wallDataMsg.walls.push_back(wallData);
        }

        // Publish all mapped walls
        pubAllWalls_new->publish(wallDataMsg);
    }
}

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    for (auto &keyframe : keyframe_vec)
        keyframe->clearClsClouds();
}

void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, rclcpp::Time msgTime)
{
    // get the latest processed keyframe
    ORB_SLAM3::KeyFrame *thisKF = nullptr;
    for (int i = keyframe_vec.size() - 1; i >= 0; i--)
    {
        if (keyframe_vec[i]->getClsCloudPtrs().size() > 0)
        {
            thisKF = keyframe_vec[i];
            // clear all clsClouds from the keyframes prior to the index i
            for (int j = 0; j < i; j++)
                keyframe_vec[j]->clearClsClouds();
            break;
        }
    }
    if (thisKF == nullptr)
        return;

    // get the class specific pointclouds from this keyframe
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs = thisKF->getClsCloudPtrs();

    // create a new pointcloud with aggregated points from all classes but with class-specific colors
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (long unsigned int i = 0; i < clsCloudPtrs.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clsCloud = clsCloudPtrs[i];
        for (long unsigned int j = 0; j < clsCloud->points.size(); j++)
        {
            pcl::PointXYZRGBA point = clsCloud->points[j];
            switch (i)
            {
            case 0: // Ground is green
                point.r = 0;
                point.g = 255;
                point.b = 0;
                break;
            case 1: // Wall is red
                point.r = 255;
                point.g = 0;
                point.b = 0;
                break;
            }
            aggregatedCloud->push_back(point);
        }
    }
    aggregatedCloud->header = clsCloudPtrs[0]->header;
    thisKF->clearClsClouds();

    // create a new pointcloud2 message from the transformed and aggregated pointcloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregatedCloud, cloud_msg);

    // publish the pointcloud to be seen at the plane frame
    cloud_msg.header.frame_id = frameCamera;
    pubSegmentedPointcloud->publish(cloud_msg);
}

void publishTrackingImage(cv::Mat image, rclcpp::Time msgTime)
{
    std_msgs::msg::Header header;
    header.stamp = msgTime;
    header.frame_id = frameWorld;
    const sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pubTrackingImage->publish(rendered_image_msg);
}

void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *> trackedMapPoints, rclcpp::Time msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud = mapPointToPointcloud(trackedMapPoints, msgTime);
    pubTrackedMappoints->publish(cloud);
}

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints, rclcpp::Time msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud = mapPointToPointcloud(allMapPoints, msgTime);
    pubAllMappoints->publish(cloud);
}

void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, rclcpp::Time msgTime)
{
    sort(keyframe_vec.begin(), keyframe_vec.end(), ORB_SLAM3::KeyFrame::lId);
    if (keyframe_vec.size() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = frameWorld;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration::from_seconds(0);
    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    visualization_msgs::msg::Marker kf_lines;
    kf_lines.id = 1;
    kf_lines.color.a = 0.15;
    kf_lines.color.r = 0.0;
    kf_lines.color.g = 0.0;
    kf_lines.color.b = 0.0;
    kf_lines.scale.x = 0.003;
    kf_lines.scale.y = 0.003;
    kf_lines.scale.z = 0.003;
    kf_lines.action = kf_lines.ADD;
    kf_lines.ns = "kf_lines";
    kf_lines.lifetime = rclcpp::Duration::from_seconds(0);
    kf_lines.header.stamp = rclcpp::Clock().now();
    kf_lines.header.frame_id = frameWorld;
    kf_lines.type = visualization_msgs::msg::Marker::LINE_LIST;

    nav_msgs::msg::Path kf_list;
    kf_list.header.frame_id = frameWorld;
    kf_list.header.stamp = msgTime;

    for (auto &keyframe : keyframe_vec)
    {
        geometry_msgs::msg::Point kf_marker;

        Sophus::SE3f kf_pose = pSLAM->GetKeyFramePose(keyframe);
        kf_marker.x = kf_pose.translation().x();
        kf_marker.y = kf_pose.translation().y();
        kf_marker.z = kf_pose.translation().z();
        kf_markers.points.push_back(kf_marker);

        // Populate the keyframe list
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Time(keyframe->mTimeStamp);
        pose.header.frame_id = frameWorld;
        pose.pose.position.x = kf_pose.translation().x();
        pose.pose.position.y = kf_pose.translation().y();
        pose.pose.position.z = kf_pose.translation().z();
        pose.pose.orientation.w = kf_pose.unit_quaternion().w();
        pose.pose.orientation.x = kf_pose.unit_quaternion().x();
        pose.pose.orientation.y = kf_pose.unit_quaternion().y();
        pose.pose.orientation.z = kf_pose.unit_quaternion().z();
        kf_list.poses.push_back(pose);
    }

    markerArray.markers.push_back(kf_markers);
    pubKeyFrameMarker->publish(markerArray);
    pubKeyFrameList->publish(kf_list);
}

void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *> markers, rclcpp::Time msgTime)
{
    int numMarkers = markers.size();
    if (numMarkers == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.resize(numMarkers);

    for (int idx = 0; idx < numMarkers; idx++)
    {
        visualization_msgs::msg::Marker fiducial_marker;
        Sophus::SE3f markerPose = markers[idx]->getGlobalPose();

        fiducial_marker.color.a = 0;
        fiducial_marker.scale.x = 0.2;
        fiducial_marker.scale.y = 0.2;
        fiducial_marker.scale.z = 0.2;
        fiducial_marker.ns = "fiducial_markers";
        fiducial_marker.lifetime = rclcpp::Duration::from_seconds(0);
        fiducial_marker.action = fiducial_marker.ADD;
        fiducial_marker.id = markerArray.markers.size();
        fiducial_marker.header.stamp = rclcpp::Clock().now(); // rclcpp::Time().now();
        fiducial_marker.mesh_use_embedded_materials = true;
        fiducial_marker.header.frame_id = frameBC;
        fiducial_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        fiducial_marker.mesh_resource =
            "package://vs_graphs/config/Assets/aruco_marker.dae";

        fiducial_marker.pose.position.x = markerPose.translation().x();
        fiducial_marker.pose.position.y = markerPose.translation().y();
        fiducial_marker.pose.position.z = markerPose.translation().z();
        fiducial_marker.pose.orientation.x = markerPose.unit_quaternion().x();
        fiducial_marker.pose.orientation.y = markerPose.unit_quaternion().y();
        fiducial_marker.pose.orientation.z = markerPose.unit_quaternion().z();
        fiducial_marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(fiducial_marker);
    }

    pubFiducialMarker->publish(markerArray);
}

void publishDoors(std::vector<ORB_SLAM3::Door *> doors, rclcpp::Time msgTime)
{
    // If there are no doors, return
    int numDoors = doors.size();
    if (numDoors == 0)
        return;

    // Variables
    visualization_msgs::msg::MarkerArray doorArray;
    doorArray.markers.resize(numDoors);

    for (int idx = 0; idx < numDoors; idx++)
    {
        Sophus::SE3f doorPose = doors[idx]->getGlobalPose();
        visualization_msgs::msg::Marker door, doorLines, doorLabel;

        // Door values
        door.color.a = 0;
        door.ns = "doors";
        door.scale.x = 0.5;
        door.scale.y = 0.5;
        door.scale.z = 0.5;
        door.action = door.ADD;
        door.lifetime = rclcpp::Duration::from_seconds(0);
        door.id = doorArray.markers.size();
        door.header.stamp = rclcpp::Clock().now(); // rclcpp::Time().now();
        door.mesh_use_embedded_materials = true;
        door.header.frame_id = frameBC;
        door.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        door.mesh_resource =
            "package://vs_graphs/config/Assets/door.dae";

        // Rotation and displacement for better visualization
        Sophus::SE3f rotatedDoorPose = doorPose * Sophus::SE3f::rotX(-M_PI_2);
        rotatedDoorPose.translation().y() -= 1.0;
        door.pose.position.x = rotatedDoorPose.translation().x();
        door.pose.position.y = rotatedDoorPose.translation().y();
        door.pose.position.z = rotatedDoorPose.translation().z();
        door.pose.orientation.x = rotatedDoorPose.unit_quaternion().x();
        door.pose.orientation.y = rotatedDoorPose.unit_quaternion().y();
        door.pose.orientation.z = rotatedDoorPose.unit_quaternion().z();
        door.pose.orientation.w = rotatedDoorPose.unit_quaternion().w();
        doorArray.markers.push_back(door);

        // Door label (name)
        doorLabel.color.a = 1;
        doorLabel.color.r = 0;
        doorLabel.color.g = 0;
        doorLabel.color.b = 0;
        doorLabel.scale.z = 0.2;
        doorLabel.ns = "doorLabel";
        doorLabel.action = doorLabel.ADD;
        doorLabel.lifetime = rclcpp::Duration::from_seconds(0);
        doorLabel.text = doors[idx]->getName();
        doorLabel.id = doorArray.markers.size();
        doorLabel.header.stamp = rclcpp::Clock().now(); // rclcpp::Time().now();
        doorLabel.header.frame_id = frameBC;
        doorLabel.pose.position.x = door.pose.position.x;
        doorLabel.pose.position.z = door.pose.position.z;
        doorLabel.pose.position.y = door.pose.position.y - 1.2;
        doorLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        doorArray.markers.push_back(doorLabel);

        // Door to points connection line
        doorLines.color.a = 0.5;
        doorLines.color.r = 0.0;
        doorLines.color.g = 0.0;
        doorLines.color.b = 0.0;
        doorLines.scale.x = 0.005;
        doorLines.scale.y = 0.005;
        doorLines.scale.z = 0.005;
        doorLines.ns = "doorLines";
        doorLines.action = doorLines.ADD;
        doorLines.lifetime = rclcpp::Duration::from_seconds(0);
        doorLines.id = doorArray.markers.size();
        doorLines.header.stamp = rclcpp::Clock().now(); // rclcpp::Time().now();
        doorLines.header.frame_id = frameBC;
        doorLines.type = visualization_msgs::msg::Marker::LINE_LIST;

        geometry_msgs::msg::Point point1;
        point1.x = doors[idx]->getMarker()->getGlobalPose().translation().x();
        point1.y = doors[idx]->getMarker()->getGlobalPose().translation().y();
        point1.z = doors[idx]->getMarker()->getGlobalPose().translation().z();
        doorLines.points.push_back(point1);

        geometry_msgs::msg::Point point2;
        point2.x = rotatedDoorPose.translation().x();
        point2.y = rotatedDoorPose.translation().y();
        point2.z = rotatedDoorPose.translation().z();
        doorLines.points.push_back(point2);

        doorArray.markers.push_back(doorLines);
    }

    pubDoor->publish(doorArray);
}

void publishPlanes(std::vector<ORB_SLAM3::Plane *> planes, rclcpp::Time msgTime)
{
    // Publish the planes, if any
    int numPlanes = planes.size();
    if (numPlanes == 0)
        return;

    // Check if sufficient time has passed since the last plane publication
    if ((msgTime - lastPlanePublishTime).seconds() < 3.0)
        return;

    lastPlanePublishTime = msgTime;

    // Variables
    visualization_msgs::msg::MarkerArray planeLabelArray;
    planeLabelArray.markers.resize(numPlanes);
    visualization_msgs::msg::Marker planeLabel, planeNormal;
    geometry_msgs::msg::Point normalStartPoint, normalEndPoint;

    // Aggregate pointcloud XYZRGB for all planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Loop through all the planes
    for (const auto &plane : planes)
    {
        // Variables
        std::vector<uint8_t> color = plane->getColor();
        Eigen::Vector3f centroid = plane->getCentroid();
        Eigen::Vector3d normal = plane->getGlobalEquation().normal();
        const std::string planeLabelText = "Plane#" + std::to_string(plane->getId());

        // If the plane is undefined, skip it
        ORB_SLAM3::Plane::planeVariant planeType = plane->getPlaneType();
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
            continue;

        // Get the point clouds for the plane
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeClouds = plane->getMapClouds();
        if (planeClouds == nullptr || planeClouds->empty())
            continue;

        Eigen::Vector4d planeCoeffs = plane->getGlobalEquation().coeffs();
        for (const auto &point : planeClouds->points)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x;
            newPoint.y = point.y;
            newPoint.z = point.z;
            newPoint.r = point.r;
            newPoint.g = point.g;
            newPoint.b = point.b;

            // Override color according to type of plane
            if (colorPointcloud)
            {
                newPoint.r = color[0];
                newPoint.g = color[1];
                newPoint.b = color[2];
            }

            // Add the point to the aggregated cloud
            aggregatedCloud->push_back(newPoint);
        }

        // Print the plane ID on the center of the plane
        planeLabel.color.a = 1.0;
        planeLabel.scale.z = 0.2;
        planeLabel.ns = "planeLabel";
        planeLabel.id = plane->getId();
        planeLabel.text = planeLabelText;
        planeLabel.header.stamp = msgTime;
        planeLabel.action = planeLabel.ADD;
        planeLabel.header.frame_id = frameBC;
        planeLabel.color.r = color[0] / 255.0;
        planeLabel.color.g = color[1] / 255.0;
        planeLabel.color.b = color[2] / 255.0;
        planeLabel.pose.position.x = centroid.x();
        planeLabel.pose.position.z = centroid.z();
        planeLabel.pose.position.y = centroid.y() - 1.5;
        planeLabel.lifetime = rclcpp::Duration::from_seconds(0);
        planeLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        // Create a marker for the plane normal (as an arrow)
        planeNormal.color.a = 1.0;
        planeNormal.scale.x = 0.01; // Shaft diameter
        planeNormal.scale.y = 0.05; // Arrowhead diameter
        planeNormal.scale.z = 0.05; // Arrowhead length
        planeNormal.ns = "planeNormal";
        planeNormal.header.stamp = msgTime;
        planeNormal.header.frame_id = frameBC;
        planeNormal.color.r = color[0] / 255.0;
        planeNormal.color.g = color[1] / 255.0;
        planeNormal.color.b = color[2] / 255.0;
        planeNormal.id = plane->getId() + numPlanes;
        planeNormal.lifetime = rclcpp::Duration::from_seconds(0);
        planeNormal.type = visualization_msgs::msg::Marker::ARROW;

        // Clear previous points from planeNormal
        planeNormal.points.clear();

        // Set the arrow's start and end points
        normalStartPoint.x = centroid.x();
        normalStartPoint.y = centroid.y();
        normalStartPoint.z = centroid.z();
        normalEndPoint.x = normalStartPoint.x + normal.x() * 0.2;
        normalEndPoint.y = normalStartPoint.y + normal.y() * 0.2;
        normalEndPoint.z = normalStartPoint.z + normal.z() * 0.2;

        planeNormal.points.push_back(normalStartPoint);
        planeNormal.points.push_back(normalEndPoint);

        // Add the normal marker to the marker array
        planeLabelArray.markers.push_back(planeLabel);
        planeLabelArray.markers.push_back(planeNormal);
    }

    if (aggregatedCloud->empty())
        return;

    // Convert the aggregated pointcloud to a pointcloud2 message
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*aggregatedCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = frameBC;

    // Publish the point cloud
    pubBuildingComponents->publish(cloudMsg);
    pubPlaneLabel->publish(planeLabelArray);
}

void publishStructuralElements(std::vector<ORB_SLAM3::Room *> rooms,
                               std::vector<ORB_SLAM3::Floor *> floors, rclcpp::Time msgTime)
{
    // Publish rooms, if any
    int numRooms = rooms.size();
    int numFloors = floors.size();

    // If there are no rooms or floors, return
    if (numRooms == 0 && numFloors == 0)
        return;

    // Visualization markers
    visualization_msgs::msg::MarkerArray roomArray, floorArray;
    roomArray.markers.resize(numRooms);
    floorArray.markers.resize(numFloors);

    // Publish rooms, if any
    if (numRooms > 0)
    {
        for (int idx = 0; idx < numRooms; idx++)
        {
            // Variables
            string roomName = rooms[idx]->getName();

            // Create color based on room type
            std::vector<double> color = {0.5, 0.1, 1.0};
            if (rooms[idx]->getIsCorridor())
                color = {0.6, 0.0, 0.3};

            Eigen::Vector3d centroid = rooms[idx]->getRoomCenter();
            visualization_msgs::msg::Marker room, roomWallLine, roomDoorLine, roomMarkerLine, roomLabel;

            // Room values
            room.ns = "room";
            room.scale.x = 0.3;
            room.scale.y = 0.3;
            room.scale.z = 0.3;
            room.color.a = 1.0;
            room.action = room.ADD;
            room.color.r = color[0];
            room.color.g = color[1];
            room.color.b = color[2];
            room.header.stamp = msgTime;
            room.header.frame_id = frameSE;
            room.id = roomArray.markers.size();
            room.mesh_use_embedded_materials = true;
            room.lifetime = rclcpp::Duration::from_seconds(0);
            room.type = visualization_msgs::msg::Marker::CUBE;

            // Rotation and displacement of the room for better visualization
            room.pose.orientation.x = 0.0;
            room.pose.orientation.y = 0.0;
            room.pose.orientation.z = 0.0;
            room.pose.orientation.w = 1.0;
            room.pose.position.x = centroid.x();
            room.pose.position.y = centroid.y();
            room.pose.position.z = centroid.z();
            roomArray.markers.push_back(room);

            // Room label (name)
            roomLabel.color.a = 1;
            roomLabel.color.r = 0;
            roomLabel.color.g = 0;
            roomLabel.color.b = 0;
            roomLabel.scale.z = 0.2;
            roomLabel.text = roomName;
            roomLabel.ns = "roomLabel";
            roomLabel.action = roomLabel.ADD;
            roomLabel.header.stamp = msgTime;
            roomLabel.header.frame_id = frameSE;
            roomLabel.id = roomArray.markers.size();
            roomLabel.pose.position.x = centroid.x();
            roomLabel.pose.position.z = centroid.z();
            roomLabel.pose.position.y = centroid.y() - 0.7;
            roomLabel.lifetime = rclcpp::Duration::from_seconds(0);
            roomLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            roomArray.markers.push_back(roomLabel);

            // Room to Plane (Wall) connection line
            roomWallLine.color.a = 0.8;
            roomWallLine.color.r = 0.0;
            roomWallLine.color.g = 0.0;
            roomWallLine.color.b = 0.0;
            roomWallLine.scale.x = 0.04;
            roomWallLine.scale.y = 0.04;
            roomWallLine.scale.z = 0.04;
            roomWallLine.ns = "room_wall_lines";
            roomWallLine.header.stamp = msgTime;
            roomWallLine.action = roomWallLine.ADD;
            roomWallLine.header.frame_id = frameWorld;
            roomWallLine.id = roomArray.markers.size();
            roomWallLine.lifetime = rclcpp::Duration::from_seconds(0);
            roomWallLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Room to Door connection line
            roomDoorLine.color.a = 0.5;
            roomDoorLine.color.r = 0.0;
            roomDoorLine.color.g = 0.0;
            roomDoorLine.color.b = 0.0;
            roomDoorLine.scale.x = 0.005;
            roomDoorLine.scale.y = 0.005;
            roomDoorLine.scale.z = 0.005;
            roomDoorLine.ns = "room_door_lines";
            roomDoorLine.header.stamp = msgTime;
            roomDoorLine.action = roomDoorLine.ADD;
            roomDoorLine.header.frame_id = frameWorld;
            roomDoorLine.id = roomArray.markers.size() + 1;
            roomDoorLine.lifetime = rclcpp::Duration::from_seconds(0);
            roomDoorLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Room to Marker connection line
            roomMarkerLine.color.a = 0.5;
            roomMarkerLine.color.r = 0.0;
            roomMarkerLine.color.g = 0.0;
            roomMarkerLine.color.b = 0.0;
            roomMarkerLine.scale.x = 0.005;
            roomMarkerLine.scale.y = 0.005;
            roomMarkerLine.scale.z = 0.005;
            roomMarkerLine.header.stamp = msgTime;
            roomMarkerLine.ns = "room_marker_lines";
            roomMarkerLine.action = roomMarkerLine.ADD;
            roomMarkerLine.header.frame_id = frameWorld;
            roomMarkerLine.id = roomArray.markers.size() + 1;
            roomMarkerLine.lifetime = rclcpp::Duration::from_seconds(0);
            roomMarkerLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Get the room center in the world frame
            // tf::Stamped<tf::Point> roomPoint, roomPointTransformed;

            // roomPoint.setX(centroid.x());
            // roomPoint.setY(centroid.y());
            // roomPoint.setZ(centroid.z());
            // roomPoint.frame_id_ = frameSE;
            // transformListener->transformPoint(frameWorld, ros::Time(0), roomPoint,
            //                                   frameSE, roomPointTransformed);

            geometry_msgs::msg::PointStamped roomPoint, roomPointTransformed;
            roomPoint.header.stamp = msgTime;
            roomPoint.point.x = centroid.x();
            roomPoint.point.y = centroid.y();
            roomPoint.point.z = centroid.z();
            roomPoint.header.frame_id = frameSE;

            try
            {
                // Use tfBuffer_ for the transform, with timeout 100ms
                // auto tf_stamped = tfBuffer_->lookupTransform(
                //     frameWorld, frameSE, tf2::TimePointZero, rclcpp::Duration::from_seconds(0.1));
                auto tf_stamped = tfBuffer_->lookupTransform(
                    frameWorld, frameSE, msgTime, rclcpp::Duration::from_seconds(0.1));
                tf2::doTransform(roomPoint, roomPointTransformed, tf_stamped);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform room center: %s", ex.what());
                roomPointTransformed = roomPoint; // fallback: use original point
            }

            // Room to Plane (Wall) connection line
            for (const auto wall : rooms[idx]->getWalls())
            {
                geometry_msgs::msg::Point pointRoom, pointWall;
                geometry_msgs::msg::PointStamped wallPoint, wallPointTransformed;

                pointRoom.x = roomPointTransformed.point.x;
                pointRoom.y = roomPointTransformed.point.y;
                pointRoom.z = roomPointTransformed.point.z;
                roomWallLine.points.push_back(pointRoom);

                wallPoint.header.stamp = msgTime;
                wallPoint.header.frame_id = frameBC;
                wallPoint.point.x = wall->getCentroid().x();
                wallPoint.point.y = wall->getCentroid().y();
                wallPoint.point.z = wall->getCentroid().z();

                try
                {
                    auto tf_stamped = tfBuffer_->lookupTransform(
                        frameWorld, frameBC, tf2::TimePointZero, tf2::durationFromSec(0.1));
                    tf2::doTransform(wallPoint, wallPointTransformed, tf_stamped);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform wall centroid: %s", ex.what());
                    wallPointTransformed = wallPoint;
                }

                pointWall.x = wallPointTransformed.point.x;
                pointWall.y = wallPointTransformed.point.y;
                pointWall.z = wallPointTransformed.point.z;
                roomWallLine.points.push_back(pointWall);
            }

            // Room to Door connection line
            for (const auto door : rooms[idx]->getDoors())
            {
                geometry_msgs::msg::Point pointRoom, pointDoor;
                geometry_msgs::msg::PointStamped doorPoint, doorPointTransformed;

                pointRoom.x = roomPointTransformed.point.x;
                pointRoom.y = roomPointTransformed.point.y;
                pointRoom.z = roomPointTransformed.point.z;
                roomDoorLine.points.push_back(pointRoom);

                doorPoint.header.stamp = msgTime;
                doorPoint.header.frame_id = frameBC;
                doorPoint.point.x = door->getGlobalPose().translation()(0);
                doorPoint.point.y = door->getGlobalPose().translation()(1);
                doorPoint.point.z = door->getGlobalPose().translation()(2);

                try
                {
                    auto tf_stamped = tfBuffer_->lookupTransform(
                        frameWorld, frameBC, tf2::TimePointZero, tf2::durationFromSec(0.1));
                    tf2::doTransform(doorPoint, doorPointTransformed, tf_stamped);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform door centroid: %s", ex.what());
                    doorPointTransformed = doorPoint;
                }

                pointDoor.x = doorPointTransformed.point.x;
                pointDoor.y = doorPointTransformed.point.y - 2.0;
                pointDoor.z = doorPointTransformed.point.z;
                roomDoorLine.points.push_back(pointDoor);
            }

            // Room to Marker connection line
            geometry_msgs::msg::Point pointRoom, pointMarker;
            ORB_SLAM3::Marker *metaMarker = rooms[idx]->getMetaMarker();

            pointRoom.x = roomPointTransformed.point.x;
            pointRoom.y = roomPointTransformed.point.y;
            pointRoom.z = roomPointTransformed.point.z;
            roomMarkerLine.points.push_back(pointRoom);
            // for (const auto door : rooms[idx]->getDoors())
            // {
            //     geometry_msgs::Point pointRoom, pointDoor;
            //     tf::Stamped<tf::Point> pointDoorInit, pointDoorTransform;

            //     pointRoom.x = roomPointTransformed.x();
            //     pointRoom.y = roomPointTransformed.y();
            //     pointRoom.z = roomPointTransformed.z();
            //     roomDoorLine.points.push_back(pointRoom);

            //     pointDoorInit.frame_id_ = frameBC;
            //     pointDoorInit.setX(door->getGlobalPose().translation()(0));
            //     pointDoorInit.setY(door->getGlobalPose().translation()(1));
            //     pointDoorInit.setZ(door->getGlobalPose().translation()(2));
            //     transformListener->transformPoint(frameWorld, ros::Time(0), pointDoorInit,
            //                                       frameBC, pointDoorTransform);

            //     pointDoor.x = pointDoorTransform.x();
            //     pointDoor.z = pointDoorTransform.z();
            //     pointDoor.y = pointDoorTransform.y() - 2.0;
            //     roomDoorLine.points.push_back(pointDoor);
            // }

            // // Room to Marker connection line
            // geometry_msgs::Point pointRoom, pointMarker;
            // tf::Stamped<tf::Point> pointMarkerInit, pointMarkerTransform;
            // ORB_SLAM3::Marker *metaMarker = rooms[idx]->getMetaMarker();

            // pointRoom.x = roomPointTransformed.x();
            // pointRoom.y = roomPointTransformed.y();
            // pointRoom.z = roomPointTransformed.z();
            // roomMarkerLine.points.push_back(pointRoom);

            // In free space-based room candidate detection, the metaMarker is not available
            if (metaMarker != nullptr)
            {
                geometry_msgs::msg::PointStamped pointMarkerInit, pointMarkerTransform;
                pointMarkerInit.header.stamp = msgTime;
                pointMarkerInit.header.frame_id = frameBC;
                pointMarkerInit.point.x = metaMarker->getGlobalPose().translation()(0);
                pointMarkerInit.point.y = metaMarker->getGlobalPose().translation()(1);
                pointMarkerInit.point.z = metaMarker->getGlobalPose().translation()(2);

                try
                {
                    auto tf_stamped = tfBuffer_->lookupTransform(
                        frameWorld, frameBC, tf2::TimePointZero, tf2::durationFromSec(0.1));
                    tf2::doTransform(pointMarkerInit, pointMarkerTransform, tf_stamped);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform marker centroid: %s", ex.what());
                    pointMarkerTransform = pointMarkerInit;
                }

                geometry_msgs::msg::Point pointMarker;
                pointMarker.x = pointMarkerTransform.point.x;
                pointMarker.y = pointMarkerTransform.point.y;
                pointMarker.z = pointMarkerTransform.point.z;
                roomMarkerLine.points.push_back(pointMarker);
            }
            // if (metaMarker != nullptr)
            // {
            //     pointMarkerInit.frame_id_ = frameBC;
            //     pointMarkerInit.setX(metaMarker->getGlobalPose().translation()(0));
            //     pointMarkerInit.setY(metaMarker->getGlobalPose().translation()(1));
            //     pointMarkerInit.setZ(metaMarker->getGlobalPose().translation()(2));
            //     transformListener->transformPoint(frameWorld, ros::Time(0), pointMarkerInit,
            //                                       frameBC, pointMarkerTransform);

            //     pointMarker.x = pointMarkerTransform.x();
            //     pointMarker.z = pointMarkerTransform.z();
            //     pointMarker.y = pointMarkerTransform.y();
            //     roomMarkerLine.points.push_back(pointMarker);
            // }

            // Add items to the roomArray
            roomArray.markers.push_back(roomWallLine);
            roomArray.markers.push_back(roomDoorLine);
            roomArray.markers.push_back(roomMarkerLine);
        }

        pubStructuralElements->publish(roomArray);
    }

    // Publish floors, if any
    if (numFloors > 0)
    {
        // Variables
        double offst = -2.0; // Visualization offset
        std::vector<double> color = {0.3, 0.6, 0.7};
        visualization_msgs::msg::Marker floorMarker, floorRoomLine;

        // Loop through all the floors
        for (int floorId = 0; floorId < numFloors; floorId++)
        {
            // Variables
            Eigen::Vector3d floorCentroid = floors[floorId]->getCentroid();

            // Floor marker (cube)
            floorMarker.id = floorId;
            floorMarker.ns = "floors";
            floorMarker.scale.x = 0.4;
            floorMarker.scale.y = 0.4;
            floorMarker.scale.z = 0.4;
            floorMarker.color.a = 1.0;
            floorMarker.color.r = color[0];
            floorMarker.color.g = color[1];
            floorMarker.color.b = color[2];
            floorMarker.header.stamp = msgTime;
            floorMarker.action = floorMarker.ADD;
            floorMarker.pose.orientation.x = 0.0;
            floorMarker.pose.orientation.y = 0.0;
            floorMarker.pose.orientation.z = 0.0;
            floorMarker.pose.orientation.w = 1.0;
            floorMarker.header.frame_id = frameSE;
            floorMarker.pose.position.x = floorCentroid.x();
            floorMarker.pose.position.z = floorCentroid.z();
            floorMarker.pose.position.y = floorCentroid.y() + offst;
            floorMarker.lifetime = rclcpp::Duration::from_seconds(0);
            floorMarker.type = visualization_msgs::msg::Marker::CUBE;

            // Floor to Room connection line
            floorRoomLine.color.a = 0.9;
            floorRoomLine.color.r = 0.0;
            floorRoomLine.color.g = 0.0;
            floorRoomLine.color.b = 0.0;
            floorRoomLine.scale.x = 0.05;
            floorRoomLine.scale.y = 0.05;
            floorRoomLine.scale.z = 0.05;
            floorRoomLine.ns = "floor_room_edge";
            floorRoomLine.header.stamp = msgTime;
            floorRoomLine.header.frame_id = frameSE;
            floorRoomLine.action = floorRoomLine.ADD;
            floorRoomLine.id = roomArray.markers.size();
            floorRoomLine.lifetime = rclcpp::Duration::from_seconds(0);
            floorRoomLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Create point for the floor centroid in the world frame
            geometry_msgs::msg::PointStamped floorPoint, floorPointT;
            floorPoint.header.stamp = msgTime;
            floorPoint.point.x = floorCentroid.x();
            floorPoint.point.y = floorCentroid.y();
            floorPoint.point.z = floorCentroid.z();
            floorPoint.header.frame_id = frameSE;

            // Connect the floor to its rooms
            for (const auto room : floors[floorId]->getRooms())
            {
                geometry_msgs::msg::Point pFloor, pRoom;
                geometry_msgs::msg::PointStamped roomPoint, roomPointT;

                pFloor.x = floorPoint.point.x;
                pFloor.z = floorPoint.point.z;
                pFloor.y = floorPoint.point.y + offst;
                floorRoomLine.points.push_back(pFloor);

                roomPoint.header.stamp = msgTime;
                roomPoint.header.frame_id = frameBC;
                roomPoint.point.x = room->getRoomCenter().x();
                roomPoint.point.y = room->getRoomCenter().y();
                roomPoint.point.z = room->getRoomCenter().z();

                pRoom.x = roomPoint.point.x;
                pRoom.y = roomPoint.point.y;
                pRoom.z = roomPoint.point.z;
                floorRoomLine.points.push_back(pRoom);
            }

            // Add the floor marker
            floorArray.markers.push_back(floorMarker);
            floorArray.markers.push_back(floorRoomLine);
        }

        pubStructuralElements->publish(floorArray);
    }
}

sensor_msgs::msg::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints, rclcpp::Time msgTime)
{
    const int numChannels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    std::string channelId[] = {"x", "y", "z"};

    // Set the attributes of the point cloud
    cloud.header.stamp = msgTime;
    cloud.header.frame_id = frameWorld;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    cloud.width = mapPoints.size();
    cloud.point_step = numChannels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count = 1;
        cloud.fields[idx].name = channelId[idx];
        cloud.fields[idx].offset = idx * sizeof(float);
        cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
    {
        if (mapPoints[idx] && !mapPoints[idx]->isBad())
        {
            Eigen::Vector3d P3Dw = mapPoints[idx]->GetWorldPos().cast<double>();
            tf2::Vector3 pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float dataArray[numChannels] = {
                pointTranslation.x(),
                pointTranslation.y(),
                pointTranslation.z()};
            memcpy(cloudDataPtr + (idx * cloud.point_step), dataArray, numChannels * sizeof(float));
        }
    }

    return cloud;
}

cv::Mat SE3fToCvMat(Sophus::SE3f data)
{
    cv::Mat cvMat;

    // Convert the Eigen matrix to OpenCV matrix
    Eigen::Matrix4f T_Eig3f = data.matrix();
    cv::eigen2cv(T_Eig3f, cvMat);

    return cvMat;
}

// tf::Transform SE3fToTFTransform(Sophus::SE3f data)
tf2::Transform SE3fToTFTransform(Sophus::SE3f data)
{
    Eigen::Matrix3f rotMatrix = data.rotationMatrix();
    Eigen::Vector3f transVector = data.translation();

    tf2::Matrix3x3 rotationTF(
        rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
        rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
        rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2));

    tf2::Vector3 translationTF(
        transVector(0),
        transVector(1),
        transVector(2));

    return tf2::Transform(rotationTF, translationTF);
}

// void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray)
// {
//     // The list of markers observed in the current frame
//     std::vector<ORB_SLAM3::Marker *> currentMarkers;

//     // Process the received marker array
//     for (const auto &marker : markerArray.markers)
//     {
//         // Access information of each passed ArUco marker
//         int markerId = marker.id;
//         double visitTime = marker.header.stamprclcpp::Time::seconds();
//         geometry_msgs::Pose markerPose = marker.pose.pose;
//         geometry_msgs::msg::Point markerPosition = markerPose.position;            // (x,y,z)
//         geometry_msgs::Quaternion markerOrientation = markerPose.orientation; // (x,y,z,w)

//         Eigen::Vector3f markerTranslation(markerPosition.x, markerPosition.y, markerPosition.z);
//         Eigen::Quaternionf markerQuaternion(markerOrientation.w, markerOrientation.x,
//                                             markerOrientation.y, markerOrientation.z);
//         Sophus::SE3f normalizedPose(markerQuaternion, markerTranslation);

//         // Create a marker object of the currently visited marker
//         ORB_SLAM3::Marker *currentMarker = new ORB_SLAM3::Marker();
//         currentMarker->setOpId(-1);
//         currentMarker->setId(markerId);
//         currentMarker->setTime(visitTime);
//         currentMarker->setMarkerInGMap(false);
//         currentMarker->setLocalPose(normalizedPose);
//         currentMarker->setMarkerType(ORB_SLAM3::Marker::markerVariant::UNKNOWN);

//         // Add it to the list of observed markers
//         currentMarkers.push_back(currentMarker);
//     }

//     // Add the new markers to the list of markers in buffer
//     if (currentMarkers.size() > 0)
//         markersBuffer.push_back(currentMarkers);
// }

std::pair<double, std::vector<ORB_SLAM3::Marker *>> findNearestMarker(double frameTimestamp)
{
    double minTimeDifference = 100;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;

    // Loop through the markersBuffer
    for (const auto &markers : markersBuffer)
    {
        double timeDifference = markers[0]->getTime() - frameTimestamp;
        if (timeDifference < minTimeDifference)
        {
            matchedMarkers = markers;
            minTimeDifference = timeDifference;
        }
    }

    return std::make_pair(minTimeDifference, matchedMarkers);
}

void setVoxbloxSkeletonCluster(const visualization_msgs::msg::MarkerArray &skeletonArray)
{
    // Reset the buffer
    skeletonClusterPoints.clear();

    for (const auto &skeleton : skeletonArray.markers)
    {
        // Take the points of the current cluster
        std::vector<Eigen::Vector3d> clusterPoints;

        // Pick only the messages starting with name "connected_vertices_[x]"
        if (skeleton.ns.compare(0, strlen("connected_vertices"), "connected_vertices") == 0)
        {
            // Skip small clusters
            if (skeleton.points.size() > ORB_SLAM3::SystemParams::GetParams()->room_seg.min_cluster_vertices)
                // Add the points of the cluster to the buffer
                for (const auto &point : skeleton.points)
                {
                    // transform from map frame to world frame
                    geometry_msgs::msg::PointStamped pointIn, pointOut;
                    pointIn.header.frame_id = frameMap;
                    pointIn.header.stamp = rclcpp::Time(0);
                    pointIn.point = point;
                    // transformListener->transformPoint(frameWorld, pointIn, pointOut);
                    try
                    {
                        auto tf_stamped = tfBuffer_->lookupTransform(
                            frameWorld, pointIn.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
                        tf2::doTransform(pointIn, pointOut, tf_stamped);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform skeleton cluster point: %s", ex.what());
                        pointOut = pointIn; // fallback: use original point
                    }
                    // Add the point to the cluster
                    Eigen::Vector3d newPoint(pointOut.point.x, pointOut.point.y, pointOut.point.z);

                    clusterPoints.push_back(newPoint);
                }

            // Add the current cluster to the skeleton cluster points buffer
            if (clusterPoints.size() > 0)
                skeletonClusterPoints.push_back(clusterPoints);
        }
    }

    // Set the cluster points to the active map
    pSLAM->setSkeletonCluster(skeletonClusterPoints);
}

void setGNNBasedRoomCandidates(const situational_graphs_msgs::msg::RoomsData &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    for (const auto &room : msgGNNRooms.rooms)
    {
        // Create a new room object
        ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

        // Set the room properties
        newRoom->setId(room.id);
        newRoom->setHasKnownLabel(false);

        // Check if corridor (if the length of room.wallIds is 2)
        bool isCorridor = (room.wall_ids.size() == 2);
        newRoom->setIsCorridor(isCorridor);

        // [TODO] use room.wallIds to fill newRoom->setWalls
        // [TODO] use room.centroid to fill newRoom->setRoomCenter

        // Add the room to the GNN candidates buffer
        gnnRoomCandidates.push_back(newRoom);

        // [TODO] Add, update, and remove the room in the GNN-based room candidates
    }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}

void setGNNBasedRoomCandidates(const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    for (const auto &room : msgGNNRooms.rooms)
    {
        // Create a new room object
        ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

        // Set the room properties
        newRoom->setId(room.id);
        newRoom->setHasKnownLabel(false);

        // Check if corridor (if the length of room.wallIds is 2)
        bool isCorridor = (room.wall_ids.size() == 2);
        newRoom->setIsCorridor(isCorridor);

        // [TODO] use room.wallIds to fill newRoom->setWalls
        // [TODO] use room.centroid to fill newRoom->setRoomCenter

        // Add the room to the GNN candidates buffer
        gnnRoomCandidates.push_back(newRoom);

        // [TODO] Add, update, and remove the room in the GNN-based room candidates
    }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}