#include "serialnode.hpp"
#include "data_pack.h"
#include "robot_interfaces/msg/robot.hpp"
#include <memory>
#include <thread>

SerialNode::SerialNode()
    : Node("driver_node") {
    exit_thread = false;
    serial      = std::make_unique<serial::Serial>("/dev/Usbttr");
    serial->open();

    serial_thread = std::make_unique<std::thread>([this]() { serial_thread_run(); });

    robot_pub = this->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);
    robot_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, std::bind(&SerialNode::legs_subscrib_cb, this, std::placeholders::_1));
}

SerialNode::~SerialNode() {
    if (serial_thread->joinable())
        serial_thread->join();
    serial->close();
}

void SerialNode::serial_thread_run() {
    robot_interfaces::msg::Robot msg;
    while (exit_thread == false) {
        serial->read((uint8_t*)legs_state, 4 * sizeof(LegPack_t));
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                msg.legs[i].joints[j].rad    = legs_target[i].leg.joint[j].rad;
                msg.legs[i].joints[j].omega  = legs_target[i].leg.joint[j].omega;
                msg.legs[i].joints[j].torque = legs_target[i].leg.joint[j].torque;
                msg.legs[i].joints[j].kp     = legs_target[i].leg.joint[j].kp;
                msg.legs[i].joints[j].kd     = legs_target[i].leg.joint[j].kd;
            }
        }
        RCLCPP_INFO(this->get_logger(),"发布电机的期望");
        robot_pub->publish(msg);
    }
}

void SerialNode::legs_subscrib_cb(const robot_interfaces::msg::Robot& msg) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            legs_target[i].leg.joint[j].rad    = msg.legs[i].joints[j].rad;
            legs_target[i].leg.joint[j].omega  = msg.legs[i].joints[j].omega;
            legs_target[i].leg.joint[j].torque = msg.legs[i].joints[j].torque;
            legs_target[i].leg.joint[j].kp     = msg.legs[i].joints[j].kp;
            legs_target[i].leg.joint[j].kd     = msg.legs[i].joints[j].kd;
        }
    }
    RCLCPP_INFO(this->get_logger(),"订阅到电机期望值，发送到电机");
    serial->write((uint8_t*)legs_target, 4 * sizeof(LegPack_t)); // 发送数据
    serial->flush();                                             // 刷新缓冲区
}
