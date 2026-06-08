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
 */

#include "common.h"

#include <atomic>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>

using namespace std;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Linearly interpolate a single accel sample to @target_time
*/
static ORB_SLAM3::IMU::Point interpolateAccel(
    const double target_time,
    const Eigen::Vector3f &cur_acc,  const double cur_t,
    const Eigen::Vector3f &prev_acc, const double prev_t,
    const Eigen::Vector3f &gyr)
{
    Eigen::Vector3f acc;

    if (prev_t == 0.0)
    {
        acc = cur_acc;
    }
    else if (target_time >= cur_t)
    {
        acc = cur_acc;
    }
    else if (target_time > prev_t)
    {
        const double factor = (target_time - prev_t) / (cur_t - prev_t);
        acc = prev_acc + static_cast<float>(factor) * (cur_acc - prev_acc);
    }
    else
    {
        acc = prev_acc;
    }

    return ORB_SLAM3::IMU::Point(
        acc.x(), acc.y(), acc.z(),
        gyr.x(), gyr.y(), gyr.z(),
        target_time);
}

// ---------------------------------------------------------------------------
// Shared IMU + Image state
// ---------------------------------------------------------------------------

struct SharedState
{
    std::mutex              mtx;
    std::condition_variable image_ready_cv;

    struct ImagePacket
    {
        cv::Mat image;
        double  timestamp = -1.0;
        double  min_marker_time_diff = std::numeric_limits<double>::max();
        std::vector<ORB_SLAM3::Marker*> matched_markers;
    };

    // Raw gyro — collected at ~200 Hz
    std::vector<double>       gyro_timestamps;
    std::vector<Eigen::Vector3f>  gyro_data;

    // Accel interpolated to gyro timestamps
    std::vector<double>       accel_timestamps_sync;
    std::vector<Eigen::Vector3f>  accel_data_sync;

    // Running accel state for interpolation 
    Eigen::Vector3f  prev_accel_data;
    double       prev_accel_timestamp = 0.0;
    Eigen::Vector3f  cur_accel_data;
    double       cur_accel_timestamp  = 0.0;

    // Image queue keeps timestamps monotonic even if tracking briefly lags.
    std::queue<ImagePacket> image_queue;
    std::size_t             max_image_queue_size = 60;
    int                     dropped_frames = 0;

    // Marker state (populated inside the image callback path)
    double                          min_marker_time_diff = std::numeric_limits<double>::max();
    std::vector<ORB_SLAM3::Marker*> matched_markers;
};

// ---------------------------------------------------------------------------
// ImuGrabber — thin ROS2 subscriber node, pushes directly into SharedStat
// ---------------------------------------------------------------------------

class ImuGrabber : public rclcpp::Node
{
public:
    explicit ImuGrabber(std::shared_ptr<SharedState> state)
        : rclcpp::Node("imu_grabber", rclcpp::NodeOptions().use_global_arguments(false))
        , state_(std::move(state))
    {}

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

private:
    std::shared_ptr<SharedState> state_;
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    const double t = rclcpp::Time(msg->header.stamp).seconds();

    const Eigen::Vector3f acc(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
    const Eigen::Vector3f gyr(msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z);

    std::lock_guard<std::mutex> lock(state_->mtx);

    // --- Gyro path (high-rate, ~200 Hz) ---
    // Just buffer; accel will be interpolated to these timestamps
    state_->gyro_data.push_back(gyr);
    state_->gyro_timestamps.push_back(t);

    // --- Accel path (lower-rate, ~60 Hz or same rate depending on IMU) ---
    // ROS Imu message carries both in the same message, so we treat every
    // message as potentially carrying a new accel sample.  We detect a new
    // accel sample by checking if t > cur_accel_timestamp
    if (t > state_->cur_accel_timestamp)
    {
        state_->prev_accel_data      = state_->cur_accel_data;
        state_->prev_accel_timestamp = state_->cur_accel_timestamp;
        state_->cur_accel_data       = acc;
        state_->cur_accel_timestamp  = t;
    }

    // Interpolate accel to cover any gyro timestamps not yet synced
    while (state_->gyro_timestamps.size() > state_->accel_timestamps_sync.size())
    {
        const int    idx         = static_cast<int>(state_->accel_timestamps_sync.size());
        const double target_time = state_->gyro_timestamps[idx];
        const Eigen::Vector3f &gyro_at_idx = state_->gyro_data[idx];

        ORB_SLAM3::IMU::Point pt = interpolateAccel(
            target_time,
            state_->cur_accel_data,  state_->cur_accel_timestamp,
            state_->prev_accel_data, state_->prev_accel_timestamp,
            gyro_at_idx);

        state_->accel_data_sync.push_back(pt.a);   // Eigen::Vector3f member
        state_->accel_timestamps_sync.push_back(target_time);
    }
}

// ---------------------------------------------------------------------------
// ImageGrabber — handles image callback and the main SyncWithImu loop
// ---------------------------------------------------------------------------

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(std::shared_ptr<SharedState> state,
                 ORB_SLAM3::System           *slam)
        : rclcpp::Node("image_grabber", rclcpp::NodeOptions().use_global_arguments(false))
        , state_(std::move(state))
        , slam_(slam)
    {}

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msg);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msg);

    // Entry point for the sync thread
    void SyncWithImu();

    std::atomic<bool> mustStop{false};
private:
    std::shared_ptr<SharedState>  state_;
    ORB_SLAM3::System            *slam_;
};

// ---------------------------------------------------------------------------
// GrabImage
// ---------------------------------------------------------------------------

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] cv_bridge exception: %s", e.what());
        return;
    }

    const double new_ts = rclcpp::Time(cv_ptr->header.stamp).seconds();

    // Resolve marker association for this timestamp.
    auto [min_diff, markers] = findNearestMarker(new_ts);

    std::unique_lock<std::mutex> lock(state_->mtx);

    if (!state_->image_queue.empty() && std::abs(state_->image_queue.back().timestamp - new_ts) < 1e-3)
        return;

    if (state_->image_queue.size() >= state_->max_image_queue_size)
    {
        state_->image_queue.pop();
        state_->dropped_frames++;
    }

    SharedState::ImagePacket packet;
    packet.image                = cv_ptr->image.clone();
    packet.timestamp            = new_ts;
    packet.min_marker_time_diff = min_diff;
    packet.matched_markers      = std::move(markers);

    state_->image_queue.push(std::move(packet));
    state_->image_ready_cv.notify_all();
}

// ---------------------------------------------------------------------------
// SyncWithImu
// ---------------------------------------------------------------------------

void ImageGrabber::SyncWithImu()
{
    while (!mustStop && !slam_->isShutDown())
    {
        cv::Mat                              im;
        double                               tIm = 0.0;
        rclcpp::Time                         msgTime;
        Eigen::Vector3f                      Wbb = Eigen::Vector3f::Zero();
        std::vector<ORB_SLAM3::IMU::Point>   vImuMeas;
        double                               min_marker_diff;
        std::vector<ORB_SLAM3::Marker*>      matched_markers;

        // --- Critical section: wait, copy, clear ---
        {
            std::unique_lock<std::mutex> lk(state_->mtx);
            state_->image_ready_cv.wait(lk, [this]{ return !state_->image_queue.empty() || mustStop; });

            if (mustStop)
                break;

            // Log dropped frames
            if (state_->dropped_frames > 1)
                RCLCPP_WARN(this->get_logger(), "%d dropped frames", state_->dropped_frames - 1);
            state_->dropped_frames = 0;

            // Copy next queued image in timestamp order.
            SharedState::ImagePacket packet = std::move(state_->image_queue.front());
            state_->image_queue.pop();

            im      = std::move(packet.image);
            tIm     = packet.timestamp;
            msgTime = rclcpp::Time(static_cast<int64_t>(tIm * 1e9));

            // Copy marker state
            min_marker_diff  = packet.min_marker_time_diff;
            matched_markers  = std::move(packet.matched_markers);

            // Build IMU measurement vector from synced accel + gyro
            const size_t n = state_->gyro_timestamps.size();
            vImuMeas.reserve(n);
            for (size_t i = 0; i < n; ++i)
            {
                if (i < state_->accel_data_sync.size())
                {
                    const Eigen::Vector3f &acc = state_->accel_data_sync[i];
                    const Eigen::Vector3f &gyr = state_->gyro_data[i];

                    vImuMeas.emplace_back(
                        acc.x(), acc.y(), acc.z(),
                        gyr.x(), gyr.y(), gyr.z(),
                        state_->gyro_timestamps[i]);

                    // Wbb = angular velocity of the IMU sample closest to image timestamp.
                    // Pick the last sample before or at tIm
                    if (state_->gyro_timestamps[i] <= tIm)
                        Wbb = state_->gyro_data[i];
                }
            }

            // Clear shared buffers
            state_->gyro_data.clear();
            state_->gyro_timestamps.clear();
            state_->accel_data_sync.clear();
            state_->accel_timestamps_sync.clear();
        }
        // Lock released — TrackMonocular runs outside lock

        if (im.empty())
            continue;

        // Image scaling
        const float imageScale = slam_->GetImageScale();
        if (imageScale != 1.f)
        {
            const int w = static_cast<int>(im.cols * imageScale);
            const int h = static_cast<int>(im.rows * imageScale);
            cv::resize(im, im, cv::Size(w, h));
        }

        // Track
        if (min_marker_diff < 0.05)
        {
            slam_->TrackMonocular(im, tIm, vImuMeas, "", matched_markers);
            markersBuffer.clear();
        }
        else
        {
            slam_->TrackMonocular(im, tIm, vImuMeas);
        }

        publishTopics(msgTime, Wbb);
    }
}

void ImageGrabber::GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage)
{
    cv_bridge::CvImageConstPtr cvImgSeg;
    const uint64_t keyFrameId = msgSegImage.key_frame_id.data;

    try
    {
        cvImgSeg = cv_bridge::toCvCopy(
            std::make_shared<sensor_msgs::msg::Image>(msgSegImage.segmented_image),
            sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] cv_bridge exception: %s", e.what());
        return;
    }

    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmented_image_probability, *pclPc2SegPrb);

    auto tuple = std::make_tuple(keyFrameId, cvImgSeg->image, pclPc2SegPrb);
    slam_->addSegmentedImage(&tuple);
}

void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");

    // --- Parameters ---
    node->declare_parameter<double>("yaw",   0.0);
    node->declare_parameter<double>("roll",  0.0); 
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<bool>("enable_pangolin",    true);
    node->declare_parameter<bool>("static_transform",   false);
    node->declare_parameter<bool>("colored_pointcloud", true);
    node->declare_parameter<bool>("publish_pointclouds", true);
    node->declare_parameter<std::string>("frame_imu",                "imu");
    node->declare_parameter<std::string>("frame_map",                "map");
    node->declare_parameter<std::string>("frame_world",              "world");
    node->declare_parameter<std::string>("frame_camera",             "camera");
    node->declare_parameter<std::string>("frame_structural_element", "struc_elem");
    node->declare_parameter<std::string>("frame_building_component", "build_comp");
    node->declare_parameter<std::string>("voc_file",         "file_not_set");
    node->declare_parameter<std::string>("settings_file",    "file_not_set");
    node->declare_parameter<std::string>("sys_params_file",  "file_not_set");

    const std::string vocFile      = node->get_parameter("voc_file").as_string();
    const std::string settingsFile = node->get_parameter("settings_file").as_string();
    const std::string sysParamsFile = node->get_parameter("sys_params_file").as_string();

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] 'voc_file' and 'settings_file' not set. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] 'sys_params_file' not set. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
 
    // Populate globals required by common.h helpers (publishTopics, etc.)
    yaw              = node->get_parameter("yaw").as_double();
    roll             = node->get_parameter("roll").as_double();
    pitch            = node->get_parameter("pitch").as_double();
    frameImu         = node->get_parameter("frame_imu").as_string();
    frameMap         = node->get_parameter("frame_map").as_string();
    frameWorld       = node->get_parameter("frame_world").as_string();
    frameCamera      = node->get_parameter("frame_camera").as_string();
    colorPointcloud  = node->get_parameter("colored_pointcloud").as_bool();
    pubPointClouds   = node->get_parameter("publish_pointclouds").as_bool();
    frameBC          = node->get_parameter("frame_building_component").as_string();
    frameSE          = node->get_parameter("frame_structural_element").as_string();
    pubStaticTransform = node->get_parameter("static_transform").as_bool();
    const bool enablePangolin = node->get_parameter("enable_pangolin").as_bool();

    // --- SLAM system (owned here, not a global) ---
    ORB_SLAM3::System slam(vocFile, settingsFile, sysParamsFile,
                           ORB_SLAM3::System::IMU_MONOCULAR, enablePangolin);
    pSLAM = &slam;  // common.h helpers still need this pointer

    // --- Shared state ---
    auto state = std::make_shared<SharedState>();

    // --- TF broadcasters (one pair, shared) ---
    tfBroadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(node);
    staticTfBroadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // --- Grabber nodes ---
    auto imugb = std::make_shared<ImuGrabber>(state);
    auto igb   = std::make_shared<ImageGrabber>(state, &slam);

    // --- QoS (sensor data profile — same as before) ---
    rclcpp::QoS sensorQos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensorQos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    sensorQos.durability(rclcpp::DurabilityPolicy::Volatile);

    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;

    auto subImu = node->create_subscription<Imu>(
        "/imu", sensorQos,
        [imugb](const Imu::ConstSharedPtr msg) { imugb->GrabImu(msg); });

    auto subImg = node->create_subscription<Image>(
        "/camera/image_raw", sensorQos,
        [igb](const Image::ConstSharedPtr msg) { igb->GrabImage(msg); });

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
    setupPublishers(node, imageTransport, node->get_name());
    setupServices(node, node->get_name());

    // --- Sync thread ---
    std::thread syncThread(&ImageGrabber::SyncWithImu, igb);

    rclcpp::spin(node);

    slam.Shutdown();
    igb->mustStop = true;
    state->image_ready_cv.notify_all();  // unblock SyncWithImu if it's waiting
    syncThread.join();

    rclcpp::shutdown();
    return 0;
}