from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare arguments (with defaults matching your declare_parameter)
    image_topic = DeclareLaunchArgument(
        "image_topic", default_value="/pylon_camera_node/image_raw"
    )
    imu_topic = DeclareLaunchArgument(
        "imu_topic", default_value="/vectornav/IMU"
    )
    odom_topic = DeclareLaunchArgument(
        "odometry_topic", default_value="/orb_slam/odom"
    )
    voc_file = DeclareLaunchArgument(
        "voc_file_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin"
    )
    settings_file = DeclareLaunchArgument(
        "settings_file_path_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/config/Monocular-Inertial/offroad.yaml"
    )
    debug_info = DeclareLaunchArgument(
        "debug_info", default_value="false"
    )
    debug_log_path = DeclareLaunchArgument(
        "debug_log_path", default_value="/workspaces/cocoro/orb_logs"
    )
    imu_time_offset = DeclareLaunchArgument(
        "imu_time_offset", default_value="0.0"
    )

    # Launch the node
    slam_node = Node(
        package="ros2_orb_slam3",
        executable="mono_inertial_cpp",
        name="stereo_inertial_slam",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "voc_file_arg": LaunchConfiguration("voc_file_arg"),
            "settings_file_path_arg": LaunchConfiguration("settings_file_path_arg"),
            "debug_info" : LaunchConfiguration("debug_info"),
            "debug_log_path" : LaunchConfiguration("debug_log_path"),
            "imu_time_offset" : LaunchConfiguration("imu_time_offset"),
        }]
    )

    return LaunchDescription([
        image_topic,
        imu_topic,
        odom_topic,
        voc_file,
        settings_file,
        debug_info,
        debug_log_path,
        imu_time_offset,
        slam_node,
        
    ])
