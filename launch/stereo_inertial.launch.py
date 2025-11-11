from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare arguments (with defaults matching your declare_parameter)
    left_image_topic = DeclareLaunchArgument("left_image_topic", default_value="/cam0/image_raw")
    right_image_topic = DeclareLaunchArgument("right_image_topic", default_value="/cam1/image_raw")
    imu_topic = DeclareLaunchArgument(
        "imu_topic", default_value="/imu0"
    )
    odom_topic = DeclareLaunchArgument(
        "odometry_topic", default_value="/orb_slam/odom"
    )
    voc_file = DeclareLaunchArgument(
        "voc_file_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin"
    )
    settings_file = DeclareLaunchArgument(
        "settings_file_path_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/config/Stereo-Inertial/euroc_mav.yaml"
    )
    debug_sync = DeclareLaunchArgument(
        "debug_sync", default_value="true"
    )

    # Launch the node
    slam_node = Node(
        package="ros2_orb_slam3",   # <-- CHANGE THIS
        executable="stereo_inertial_cpp",  # <-- CHANGE THIS (from CMake install)
        name="stereo_inertial_slam",
        output="screen",
        parameters=[{
            "left_image_topic": LaunchConfiguration("left_image_topic"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "voc_file_arg": LaunchConfiguration("voc_file_arg"),
            "settings_file_path_arg": LaunchConfiguration("settings_file_path_arg"),
            "debug_sync" : LaunchConfiguration("debug_sync"),
        }]
    )

    return LaunchDescription([
        left_image_topic,
        right_image_topic,
        imu_topic,
        odom_topic,
        voc_file,
        settings_file,
        debug_sync,
        slam_node,
        
    ])
