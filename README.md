### Assignment6 ###
# To remove building folders
    rm -r build/ install/ log/

# To run the the video streamer on the host machine
    cd Documents/cam
    python videoserver.py

# To build the packages
    cd relBot
    colcon build

# To sourcing the package
    source install/setup.bash

# To run the nodes and the program
    ros2 launch relbot_launch relbot_sequence_controller.launch.py
    ros2 launch relbot_launch relbot_cam2image.launch.py