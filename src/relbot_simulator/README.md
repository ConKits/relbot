Package relbot_simulator
-----------------------------------------------
#### Description: 
This package implements a simulated relbot. Model generated by 20-sim

#### Inputs:
/image 
        Type: sensor_msgs/msg/Image
/input/left_motor/setpoint_vel
/input/right_motor/setpoint_vel
        Type: example_interfaces/msg/Float64

#### Outputs:
/output/camera_position
        Type: geometry_msgs/msg/PointStamped
/output/moving_camera
        Type: sensor_msgs/msg/Image
/output/robot_pose
        Type: geometry_msgs/msg/PoseStamped

#### Run:
In a terminal run either of the following commands:
`ros2 run relbot_simulator relbot_simulator`
`ros2 launch relbot_launch relbot_system.launch.py`
#### Parameters:
double `image_stream_FPS` : Sets the output rate of image stream. Default = 30 FPS, which is most webcams

#### Core components: 
* `dynamics_timer_callback()`: Calls and runs the model every timestep
* `webcam_topic_callback()`: Ingests and passes on webcam image stream
* `CreateCVSubimage()`: Creates sub-image which is then published on /output/moving_camera