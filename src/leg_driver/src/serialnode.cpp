#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <robot_interfaces/msg/robot.hpp>
#include <memory>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    // 初始化状态
    exit_thread = false;

    // 先创建 publisher/subscriber，确保回调中 publish 时 publisher 已就绪
    robot_pub = this->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);

    robot_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));


    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(LegPack_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const LegPack_t* pack = reinterpret_cast<const LegPack_t*>(data);
            if (pack->pack_type == 0)  // 确认包类型正确
                publishLegState(pack); // 一旦接收，立即发布狗腿状态
        }
    });
    if(!cdc_trans->open(0x0483, 0x5740))                                // 开启USB_CDC传输接口
        exit_thread=true;

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do{
            cdc_trans->process_once();
        }while (!exit_thread);
    });


    target_send_thread=std::make_unique<std::thread>([this](){
        do{
            auto now = std::chrono::system_clock::now();
            //调试
            legs_target.leg[2].joint[2].kd=0.05f;
            cdc_trans->send_struct(legs_target);
            std::this_thread::sleep_until(now + 5ms);
        }while (!exit_thread);
    });
}

SerialNode::~SerialNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if(target_send_thread&&target_send_thread->joinable()){
        target_send_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const LegPack_t* legs_state) {
    robot_interfaces::msg::Robot msg;
    RCLCPP_INFO(this->get_logger(), "发布电机的当前状态");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            msg.legs[i].joints[j].rad    = legs_state->leg[i].joint[j].rad;
            msg.legs[i].joints[j].omega  = legs_state->leg[i].joint[j].omega;
            msg.legs[i].joints[j].torque = legs_state->leg[i].joint[j].torque;
            msg.legs[i].joints[j].kp     = legs_state->leg[i].joint[j].kp;
            msg.legs[i].joints[j].kd     = legs_state->leg[i].joint[j].kd;
            RCLCPP_INFO(this->get_logger(),"腿%d-关节%d-位置%f",i,j,legs_state->leg[i].joint[j].rad);
        }
    }
    robot_pub->publish(msg);
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Robot& msg) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            legs_target.leg[i].joint[j].rad    = msg.legs[i].joints[j].rad;
            legs_target.leg[i].joint[j].omega  = msg.legs[i].joints[j].omega;
            legs_target.leg[i].joint[j].torque = msg.legs[i].joints[j].torque;
            legs_target.leg[i].joint[j].kp     = msg.legs[i].joints[j].kp;
            legs_target.leg[i].joint[j].kd     = msg.legs[i].joints[j].kd;
        }
    }
    //cdc_trans->send_struct(legs_target); // 一旦订阅到最新的包，立即发送到下位机
    RCLCPP_INFO(this->get_logger(), "订阅到电机期望值");
}
