from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    device_ip = LaunchConfiguration("device_ip", default="192.168.137.200")
    points_port = LaunchConfiguration("points_port", default="1001")
    imu_port = LaunchConfiguration("imu_port", default="1002")
    status_port = LaunchConfiguration("status_port", default="1003")
    frame_id = LaunchConfiguration("frame_id", default="ws30_lidar")
    packet_timeout_ms = LaunchConfiguration("packet_timeout_ms", default="50")
    publish_imu = LaunchConfiguration("publish_imu", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("device_ip", default_value="192.168.137.200"),
        DeclareLaunchArgument("points_port", default_value="1001"),
        DeclareLaunchArgument("imu_port", default_value="1002"),
        DeclareLaunchArgument("status_port", default_value="1003"),
        DeclareLaunchArgument("frame_id", default_value="ws30_lidar"),
        DeclareLaunchArgument("packet_timeout_ms", default_value="50"),
        DeclareLaunchArgument("publish_imu", default_value="true"),

        Node(
            package="ws30_lidar_bridge",
            executable="ws30_lidar_node",
            name="ws30_lidar_node",
            namespace="gimbal/laser_guidance/ws30",
            output="screen",
            parameters=[{
                "device_ip": device_ip,
                "points_port": points_port,
                "imu_port": imu_port,
                "status_port": status_port,
                "frame_id": frame_id,
                "packet_timeout_ms": packet_timeout_ms,
                "publish_imu": publish_imu,
                "use_sensor_timestamp": True,
            }],
        ),
    ])
