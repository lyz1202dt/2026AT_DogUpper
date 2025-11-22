#ifndef __MYNODE_H__
#define __MYNODE_H__

#include <rclcpp/rclcpp.hpp>
#include "leg.h"
#include "step.h"
#include <rclcpp/subscription.hpp>
#include <tuple>
#include <Eigen/Dense>
#include "robot_interfaces/msg/robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class LegControl :public rclcpp::Node{
public:
    LegControl(LegParam_t &leg_param,std::string name);
    ~LegControl();
private:
    void Run_Cb();
    std::tuple<Vector3D,Vector3D,Vector3D> leg_state;
    std::tuple<Vector3D,Vector3D,Vector3D> leg_target;
    Trajectory_t trajectory;
    CycloidStep_t cycloid_step;

    Leg *leg;

    double cur_time;            //当前运行时间
    double leg_run_time;    //一个脚步的时间
    int state_flag;          //更新状态标志位

    bool step_update_flag;
    
    rclcpp::TimerBase::SharedPtr update_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
};

#endif