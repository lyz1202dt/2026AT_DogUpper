#include "serialnode.hpp"
#include "data_pack.h"
#include "package_comm.hpp"
#include "robot_interfaces/msg/robot.hpp"
#include <memory>


SerialNode::SerialNode()
    : Node("driver_node") {
    package_comm=std::make_unique<PackageComm>();
    package_comm->register_recv_cb([this](const uint8_t *data,int size){
        /*if(size==sizeof(LegPack_t))     //验证包长度，可以被视作四条腿的状态数据包
        {
            const LegPack_t *pack=reinterpret_cast<const LegPack_t*>(data);
            if(pack->pack_type==0)  //确认包类型正确
                publishLegState(pack);  //一旦接收，立即发布狗腿状态
        }*/
        /*RCLCPP_INFO(this->get_logger(),"完整的包长度%d",size);
        RCLCPP_INFO(this->get_logger(),"第1个字节%d",(int)data[0]);
        RCLCPP_INFO(this->get_logger(),"第65个字节%d",(int)data[64]);
        RCLCPP_INFO(this->get_logger(),"第100个字节%d",(int)data[99]);*/
    });

    robot_pub = this->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);
    robot_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));
}

SerialNode::~SerialNode() {

}

void SerialNode::publishLegState(const LegPack_t *legs_state) {
    robot_interfaces::msg::Robot msg;
    while (exit_thread == false) {

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                msg.legs[i].joints[j].rad    = legs_state->leg[i].joint[j].rad;
                msg.legs[i].joints[j].omega  = legs_state->leg[i].joint[j].omega;
                msg.legs[i].joints[j].torque = legs_state->leg[i].joint[j].torque;
                msg.legs[i].joints[j].kp     = legs_state->leg[i].joint[j].kp;
                msg.legs[i].joints[j].kd     = legs_state->leg[i].joint[j].kd;
            }
        }
        RCLCPP_INFO(this->get_logger(),"发布电机的当前状态");
        robot_pub->publish(msg);
    }
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Robot &msg) {
    LegPack_t legs_target;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            legs_target.leg[i].joint[j].rad    = msg.legs[i].joints[j].rad;
            legs_target.leg[i].joint[j].omega  = msg.legs[i].joints[j].omega;
            legs_target.leg[i].joint[j].torque = msg.legs[i].joints[j].torque;
            legs_target.leg[i].joint[j].kp     = msg.legs[i].joints[j].kp;
            legs_target.leg[i].joint[j].kd     = msg.legs[i].joints[j].kd;
        }
    }
    package_comm->async_send_struct(legs_target);   //一旦订阅到最新的包，立即发送到下位机
    RCLCPP_INFO(this->get_logger(),"订阅到电机期望值，发送到电机");
}
