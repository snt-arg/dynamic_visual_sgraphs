from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os.path

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            # Global arguments
            DeclareLaunchArgument("offline", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("colored_pointcloud", default_value="true"),
            DeclareLaunchArgument("visualize_segmented_scene", default_value="true"),
            # Topics
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("sensor_config", default_value="RealSense_D435i"),
            DeclareLaunchArgument(
                "rgb_image_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
            ),
            # VS-Graphs Node
            Node(
                package="orb_slam3_ros",
                executable="ros_rgbd",
                # name="orb_slam3",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("offline")},
                    {
                        "voc_file": LaunchConfiguration(
                            "voc_file",
                            default=[
                                # LaunchConfiguration(
                                #     "find_package_share", default="orb_slam3_ros"
                                # ),
                                get_package_share_directory("orb_slam3_ros"),
                                "/Vocabulary/ORBvoc.txt.bin",
                            ],
                        )
                    },
                    {
                        "settings_file": LaunchConfiguration(
                            "settings_file",
                            default=[
                                # LaunchConfiguration(
                                #     "find_package_share", default="orb_slam3_ros"
                                # ),
                                # "/home/asier/ros2_ws/install/orb_slam3_ros/share/orb_slam3_ros/",
                                get_package_share_directory("orb_slam3_ros"),
                                "/config/RGB-D/",
                                LaunchConfiguration("sensor_config"),
                                ".yaml",
                            ],
                        )
                    },
                    {
                        "sys_params_file": LaunchConfiguration(
                            "sys_params_file",
                            default=[
                                # LaunchConfiguration(
                                #     "find_package_share", default="orb_slam3_ros"
                                # ),
                                get_package_share_directory("orb_slam3_ros"),
                                "/config/system_params.yaml",
                            ],
                        )
                    },
                    {"static_transform": True},
                    {"roll": 0.0},
                    {"yaw": 1.5697},
                    {"pitch": -1.5697},
                    {"frame_map": "map"},
                    {"cam_frame_id": "camera"},
                    {"world_frame_id": "world"},
                    {"enable_pangolin": False},
                    {"publish_pointclouds": True},
                    {"colored_pointcloud": LaunchConfiguration("colored_pointcloud")},
                ],
                remappings=[
                    ("/camera/rgb/image_raw", LaunchConfiguration("rgb_image_topic")),
                    (
                        "/camera/depth_registered/image_raw",
                        LaunchConfiguration("depth_image_topic"),
                    ),
                ],
            ),
            # Static Transforms (use ROS 2 static_transform_publisher)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="bc_to_se",
                arguments=["0", "-3", "0", "0", "0", "0", "plane", "room"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_bc",
                arguments=["0", "-5", "0", "0", "0", "0", "world", "plane"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_to_camera_optical",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "camera",
                    "camera_color_optical_frame",
                ],
            ),
            # # Trajectory Path Server (if available in ROS 2)
            # Node(
            #     package="hector_trajectory_server",
            #     executable="hector_trajectory_server",
            #     name="trajectory_server_orb_slam3",
            #     namespace="orb_slam3_ros",
            #     output="screen",
            #     parameters=[
            #         {"target_frame_name": "/map"},
            #         {"source_frame_name": LaunchConfiguration("camera_frame")},
            #         {"trajectory_update_rate": 20.0},
            #         {"trajectory_publish_rate": 20.0},
            #     ],
            # ),
            # RViz
            Node(
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [
                        # LaunchConfiguration(
                        #     "find_package_share", default="orb_slam3_ros"
                        # ),
                        get_package_share_directory("orb_slam3_ros"),
                        "/config/Visualization/vsgraphs_rgbd2.rviz",
                        # get_package_share_directory("orb_slam3_ros"),
                        # "/config/Visualization/",
                        # LaunchConfiguration("vsgraphs_rgbd"),
                        # ".rviz",
                    ],
                ],
                output="screen",
            ),
            # # Depth to Point Cloud (nodelet is not used in ROS 2, use component or node directly)
            # Node(
            #     package="depth_image_proc",
            #     executable="point_cloud_xyzrgb",
            #     name="point_cloud_xyzrgb",
            #     remappings=[
            #         ("rgb/camera_info", LaunchConfiguration("rgb_camera_info_topic")),
            #         ("rgb/image_rect_color", LaunchConfiguration("rgb_image_topic")),
            #         (
            #             "depth_registered/image_rect",
            #             LaunchConfiguration("depth_image_topic"),
            #         ),
            #         ("depth_registered/points", "/camera/depth/points"),
            #     ],
            # ),
            ComposableNodeContainer(
                name="depth_image_proc_container",
                package="rclcpp_components",
                namespace="",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            (
                                "rgb/camera_info",
                                LaunchConfiguration("rgb_camera_info_topic"),
                            ),
                            (
                                "rgb/image_rect_color",
                                LaunchConfiguration("rgb_image_topic"),
                            ),
                            (
                                "depth_registered/image_rect",
                                LaunchConfiguration("depth_image_topic"),
                            ),
                            ("points", "/camera/depth/points"),
                        ],
                    ),
                ],
            ),
            # Semantic Scene Segmenter Node
            Node(
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                name="segmenter_ros",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        # LaunchConfiguration(
                        #     "find_package_share", default="segmenter_ros"
                        # ),
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),
        ]
    )
