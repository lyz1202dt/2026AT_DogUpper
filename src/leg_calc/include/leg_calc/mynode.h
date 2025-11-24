#ifndef __MYNODE_H__
#define __MYNODE_H__

#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include "leg.h"
#include "step.h"
#include "vmc.hpp"
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
#include <rclcpp/parameter.hpp>

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
    VMC* vmc;

    double leg_virtual_force;

    bool enable_vmc{false};
    double force_filter_gate{0.8};
    double joint1_kp;
    double joint1_kd;
    double joint2_kp;
    double joint2_kd;
    double joint3_kp;
    double joint3_kd;

    float leg_run_time;    //一个脚步的时间
    std::chrono::high_resolution_clock::time_point last_step_reset_time;    //上次步态重置时间点
    
    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr move_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;


    Vector3D foot_force;   //足端力矩
    Vector3D foot_pos;
    Vector3D foot_vel;

    //参数服务
    OnSetParametersCallbackHandle::SharedPtr param_server_handle;
};

#endif