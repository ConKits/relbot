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
            package='cam2image_vm2ros',
            executable='cam2image',
            parameters=["src/cam2image_vm2ros/config/cam2image.yaml"],  
        ),
        Node(
            package="my_image_processor",
            executable="image_processor"
        ),
        # Node(
        #     package="relbot_simulator",
        #     executable="relbot_simulator",
        #     name="relbot_simulator"
        #     ),
        # Node(
        #     package="relbot_sequence_controller",
        #     executable="relbot_sequence_controller",
        #     name="relbot_sequence_controller"
        # ),
        
    ])


