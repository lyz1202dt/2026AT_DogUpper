#ifndef __MYNODE_H__
#define __MYNODE_H__

#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include "leg.h"
#include "step.h"
#include <rclcpp/subscription.hpp>
#include <tuple>
#include <Eigen/Dense>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <chrono>

class LegControl :public rclcpp::Node{
public:
    LegControl(LegParam_t &leg_param,std::string name);
    ~LegControl();
private:
    void Run_Cb();
    void Show_Cb();
    std::tuple<Vector3D,Vector3D,Vector3D> leg_state;
    std::tuple<Vector3D,Vector3D,Vector3D> leg_target;
    Trajectory_t trajectory;
    CycloidStep_t cycloid_step;

    Leg *leg;

    float leg_run_time;    //一个脚步的时间
    std::chrono::high_resolution_clock::time_point last_step_reset_time;    //上次步态重置时间点
    
    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr move_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
};

#endif