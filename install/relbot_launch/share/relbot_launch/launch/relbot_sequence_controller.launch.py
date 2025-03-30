import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
      get_package_share_directory('cam2image_vm2ros'),
      'config',
      'cam2image.yaml'
      )    
    return LaunchDescription([
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator"
        ),
        Node(
            package="relbot2turtlesim",
            executable="relbot2turtlesim",
            name="relbot2turtlesim"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        ),
        Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        ),      
        Node(
            package="cam2image_vm2ros",
            executable="cam2image",
            name="fuckyou",
            parameters=[config_path]
        )   
    ])
