from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare arguments (with defaults matching your declare_parameter)
    image_topic = DeclareLaunchArgument("image_topic", default_value="/cam0/image_raw")
    odom_topic = DeclareLaunchArgument(
        "odometry_topic", default_value="/orb_slam/odom"
    )
    voc_file = DeclareLaunchArgument(
        "voc_file_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin"
    )
    settings_file = DeclareLaunchArgument(
        "settings_file_path_arg", default_value="/workspaces/cocoro/src/ros2_orb_slam3/orb_slam3/config/Monocular/EuRoC.yaml"
    )
    debug_info = DeclareLaunchArgument(
        "debug_info", default_value="false"
    )

    # Launch the node
    slam_node = Node(
        package="ros2_orb_slam3",   # <-- CHANGE THIS
        executable="mono_cpp",  # <-- CHANGE THIS (from CMake install)
        name="stereo_inertial_slam",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "voc_file_arg": LaunchConfiguration("voc_file_arg"),
            "settings_file_path_arg": LaunchConfiguration("settings_file_path_arg"),
            "debug_info" : LaunchConfiguration("debug_info"),
        }]
    )

    return LaunchDescription([
        image_topic,
        odom_topic,
        voc_file,
        settings_file,
        debug_info,
        slam_node,
        
    ])
