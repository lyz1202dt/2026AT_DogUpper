#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <fstream>
#include <iostream>


class LegMeasureNode : public rclcpp::Node {
public:
    LegMeasureNode();
    ~LegMeasureNode();

private:
    void measureCallback(const robot_interfaces::msg::Robot& msg);

    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::TimerBase::SharedPtr target_set_timer;
};

