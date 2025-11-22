#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <vector>


class LegMeasureNode : public rclcpp::Node {
public:
    LegMeasureNode();
    ~LegMeasureNode();

private:
    void measureCallback(const robot_interfaces::msg::Robot& msg);
    void startMeasure(int sample_num);
    bool saveResultToFile();

    bool start_measure;
    bool is_sampling;

    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::TimerBase::SharedPtr target_set_timer;
    std::vector<std::array<float, 6>> rad_torque_sample;
    int cur_index;
};

