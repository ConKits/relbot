
#include <cmath>
#include "steering.hpp"

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
}

void SteerRelbot::create_topics() {
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
    
    object_cordinates_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/green_object_position", 10,
            std::bind(&SteerRelbot::position_callback, this, std::placeholders::_1)) ;
}


void SteerRelbot::idle() {
    // Idle state logic
    // This method is used to set the robot to idle state when no object is detected.
    left_velocity = 0.0;
    right_velocity = 0.0;
    idleState=true;
    RCLCPP_INFO(this->get_logger(), "Idle");
}

void SteerRelbot::moveStraight() {
    // Moves the robot straight
    // This method is calculating the velocities for each wheel to move straight.
    left_velocity = -(angularVelocity * radius);
    right_velocity = angularVelocity * radius;
    RCLCPP_INFO(this->get_logger(), "Moving straight");
}

void SteerRelbot::rotate(int direction) {
    // Rotates the robot
    // This method is calculating the velocities for each wheel to rotate.
    left_velocity = direction*(-angularVelocity*(radius - (wheelDistance/2)));
    right_velocity = direction*(angularVelocity*(radius + (wheelDistance/2)));
    RCLCPP_INFO(this->get_logger(), "Rotating");
    
}

void SteerRelbot::calculate_velocity() {
    // Calculate the velocities for each wheel based on the robot's position and the object's position    
   
    if (idleState==false){   
        // Calculate the error between the robot's position and the object's position
        x_error = x_object - x_center;

            if (x_error > 0 && std::abs(x_error) > x_tol) {
                // Object is to the right of the center
                rotate(1);
                
            } 
            else if (x_error < 0 && std::abs(x_error) > x_tol) {
                // Object is to the left of the center
                rotate(-1);
            }
            else{
                if (area_object< threshold_area) {
                    // Object is far from the robot
                    moveStraight();
                } 
                else {
                    // Object is close to the robot
                    idle();
                }
            }
    }
}

    

void SteerRelbot::timer_callback() {
    // calculate velocity
    calculate_velocity();

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_topic_->publish(left_wheel);
    right_wheel_topic_->publish(right_wheel);
}

// Callback function to receive the position of the green object
// This function is called when a new message is received on the subscribed topic
void SteerRelbot::position_callback(const geometry_msgs::msg::PointStamped::SharedPtr cord){
   
    area_object=cord->point.z;
    //Check for close green objects.
    if (area_object>minimum_area){
        idleState=false;
        x_object=cord->point.x;
        y_object=cord->point.y;
        //area_object=cord->point.z;
        //RCLCPP_INFO(this->get_logger(), "Received Green Object Position -> x: %.2f, y: %.2f, area: %.2f", x_object, y_object, area_object);
    }
    else{
        idleState=true;
        x_object=0.0;
        y_object=0.0;
        //area_object=0.0;
        //RCLCPP_INFO(this->get_logger(), "No Green Object Detected");
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}
/*
Pseudo Code to control the relbot:
-Must have two states/modes, the idle mode and action mode.

-Idle mode: The logic in the idle mode must do with timing of the next values of objects cordinates.
    *Less than a sertain amount of time it should start calculate the error between the actual position and the object's.

-Action mode: In this mode the calculations of the wheels velocities are made by the given equations in the manual.
    *The checker is the error between the two spots, and if that equals zero the robot should be in the idle mode.
    *If the error is not equal zero then the robot should be in the action mode.
    *Assuming that the center of the camera/image is the 0,0 origin and then move the robot in such a way to reduce the error
    close to zero. It will be useful to use tollerance to estimate the positions.
    *So error=camera_center-object_centre. 
    ***REMINDER*** Convert object's cordinates from top-rihgt corner origin to the center of the image/camera. 
                    This can be done by adding the half size of the frame, width and height.
-Navigation Logic: If the idle sate is not true then the robot should be in the action mode.
    First check the x_error and turn respectively to position the robot at the center. Then move straight is the area is leass than the threshold.
    If the area is greater than the threshold then the robot should be in idle state.

-To Do:
    *Find the correct valus of threshold and minimum area.
    *Corect the time_constant based on the robot's speed.
    *Test the code in the simulator.
    *Test the code in the real robot.
*/