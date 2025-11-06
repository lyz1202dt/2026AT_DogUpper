from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("leg_module"),
        "leg_urdf", "leg.urdf"
    )
    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    leg_calc = Node(
        package="leg_calc",
        executable="leg_calc"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2"
        # arguments=["-d", ...]  # 可选，指定rviz配置文件
    )
    return LaunchDescription([robot_state_pub, leg_calc, rviz2])