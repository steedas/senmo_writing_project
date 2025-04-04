import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    kuka_lbr_parameters = os.path.join(
        get_package_share_directory('kinematics'),
        'config',
        'kuka_lbr.yaml'
    )
    kuka_lbr_param_node = Node(
        package='kinematics',
        namespace='kuka_lbr',
        executable='robot_param_server',
        name='param_server',
        parameters=[kuka_lbr_parameters]
    )
    ld.add_action(kuka_lbr_param_node)

    return ld