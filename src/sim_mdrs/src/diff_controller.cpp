#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class Controller : public rclcpp::Node {

private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_wheel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    double angle_data;

    #define ROVER_WHEEL_RADIUS 0.075

    #define d1 0.177 // Distance from the center to the front-left wheel along the x-axis
    #define d2 0.310 // Distance from the center to the front-left wheel along the y-axis
    #define d3 0.274 // Distance from the center to the rear wheels along the y-axis
    #define d4 0.253 // Distance from the center to the center wheel along the x-axis

    double theta = 0;
    double l = 0; // linear, angular turning radius

    double theta_front_closest;
    double theta_front_farthest;
    rclcpp::Time last_time;
    // rclcpp::Time current_time;
    double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
    double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
    double start_time, time, pre_time;

    geometry_msgs::msg::Twist pri_velocity;


    // double fl_vel, fr_vel, ml_vel, mr_vel, rl_vel, rr_vel;
    double left_vel, right_vel
    double current_dl, dl, pre_dl;
    double x_postion, y_postion;

    double FL_data, FR_data, ML_data, MR_data, RL_data, RR_data;


    bool delay_ = true;

    double dt;
    double test1;
    double test = 0;
    nav_msgs::msg::Odometry odom_msg;

public:
    Controller() : Node("controller") {
        motor_wheel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 1);

        sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&Controller::msgCallback, this, std::placeholders::_1));

        joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&Controller::jointStateCallback, this, std::placeholders::_1));

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("osr/odom", 10);

    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

        fl_vel = msg->position[5];
        fr_vel = msg->position[7];
        ml_vel = msg->position[2];
        mr_vel = msg->position[3];
        rl_vel = msg->position[8];
        rr_vel = msg->position[9];

        Odometry(theta);

    }

    void Odometry(double angle)
    {
        double v = (left_vel + right_vel) * ROVER_WHEEL_RADIUS / 2;
        double w = (right_vel - left_vel) * ROVER_WHEEL_RADIUS / d4; // d4 as track width

        dl = v * dt;
        theta += w * dt;

        x_postion += dl * cos(theta);
        y_postion += dl * sin(theta);
        
        // RCLCPP_INFO(this->get_logger(), "x_position: %f, y_position: %f", x_postion, y_postion);

        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = x_postion;
        odom_msg.pose.pose.position.y = y_postion;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, angle);

        odom_msg.pose.pose.orientation.x = quaternion.x();
        odom_msg.pose.pose.orientation.y = quaternion.y();
        odom_msg.pose.pose.orientation.z = quaternion.z();
        odom_msg.pose.pose.orientation.w = quaternion.w();

        static tf2_ros::TransformBroadcaster br(this);
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_footprint";

        transformStamped.transform.translation.x = x_postion;
        transformStamped.transform.translation.y = y_postion;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation = odom_msg.pose.pose.orientation;
        br.sendTransform(transformStamped);

        odom_pub->publish(odom_msg);

       // printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
    }

    void msgCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        if((pri_velocity.linear.x == msg->linear.x) && (pri_velocity.angular.z == msg->angular.z)) return;

        if(delay_)
        {
            delay_ = false;

            if (msg->angular.z == 0 && msg->linear.x != 0) { // linear velocity
                go_straight(msg);
                publishVelocity();
            }

            else if((msg->angular.z != 0)&&(msg->linear.x == 0))  // rotate in place
            {
                rotate_in_place(msg);
                publishVelocity();
            }

            else if((msg->angular.z == 0)&&(msg->linear.x == 0))
            {
                stop();
                publishVelocity();
            }

            delay_ = true;
        }

        pri_velocity.linear.x = msg->linear.x;
        pri_velocity.angular.z = msg->angular.z;

    }

    void stop()
    {
        FL_data = 0;
        RL_data = 0;

        ML_data = 0;
        FR_data = 0;

        RR_data = 0;
        MR_data = 0;

    }

    void go_straight(const geometry_msgs::msg::Twist::SharedPtr msg) {

        double velocity_data = msg->linear.x / ROVER_WHEEL_RADIUS;
        left_vel = velocity_data;
        right_vel = velocity_data;
    }

    void rotate_in_place(const geometry_msgs::msg::Twist::SharedPtr& msg)
    {
        double angular_velocity = msg->angular.z * (d4 / 2) / ROVER_WHEEL_RADIUS; // d4 assumed as track width
        left_vel = -angular_velocity;
        right_vel = angular_velocity;

    }

    void publishVelocity() {

        std_msgs::msg::Float64MultiArray wheel;

        wheel.data = {left_vel, right_vel, left_vel, right_vel, left_vel, right_vel};

        motor_wheel_pub->publish(wheel);
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    // Using std::chrono to create a rate object
    auto rate = std::chrono::milliseconds(300); // 200 Hz = 5ms per loop

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Manually sleep to maintain the loop rate
        std::this_thread::sleep_for(rate);
    }
    rclcpp::shutdown();
    return 0;
}
