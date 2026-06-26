import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("warehouse_patrol")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_share, "worlds", "warehouse.sdf")
    urdf_file = os.path.join(pkg_share, "urdf", "patrol_robot.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "warehouse_patrol.rviz")

    with open(urdf_file, "r", encoding="utf-8") as file:
        robot_desc = file.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    launch_rviz = LaunchConfiguration("launch_rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_desc},
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/patrol_robot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/odometry", "/odom"),
            ("/model/patrol_robot/scan", "/scan"),
        ],
    )

    odom_tf_broadcaster = Node(
        package="warehouse_patrol",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odom_topic": "/odom"},
            {"parent_frame": "odom"},
            {"child_frame": "base_link"},
        ],
    )

    safety_filter = Node(
        package="warehouse_patrol",
        executable="safety_filter",
        name="safety_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    patrol_scheduler = Node(
        package="warehouse_patrol",
        executable="patrol_scheduler",
        name="patrol_scheduler",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=f"-s -r {world_path}",
            description="Gazebo Sim args; use '-s -r <world>' for headless server mode",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the warehouse patrol display configuration",
        ),
        gazebo,
        bridge,
        odom_tf_broadcaster,
        robot_state_publisher,
        safety_filter,
        patrol_scheduler,
        rviz,
    ])
