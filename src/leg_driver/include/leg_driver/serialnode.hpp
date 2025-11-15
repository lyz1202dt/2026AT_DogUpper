#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot.hpp"
#include "data_pack.h"
#include "package_comm.hpp"
#include "mutex"


class SerialNode : public rclcpp::Node
{
public:
    SerialNode();
    ~SerialNode();

private:
    bool exit_thread;
    void legsSubscribCb(const robot_interfaces::msg::Robot &msg);
    void publishLegState(const LegPack_t *legs_state);

    std::unique_ptr<PackageComm> package_comm;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr robot_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr robot_sub;
};

#endif