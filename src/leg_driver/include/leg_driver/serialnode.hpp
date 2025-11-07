#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot.hpp"
#include "serial/serial.h"
#include "data_pack.h"
#include <memory>
#include <thread>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode();
    ~SerialNode();

    private:
    bool exit_thread;
    void legs_subscrib_cb(const robot_interfaces::msg::Robot &msg);
    void serial_thread_run();

    LegPack_t legs_target[4];
    LegPack_t legs_state[4];

    std::unique_ptr<std::thread> serial_thread;
    std::unique_ptr<serial::Serial> serial;

    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr robot_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr robot_sub;
};

#endif