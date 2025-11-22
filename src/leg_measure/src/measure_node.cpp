#include "measure_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

LegMeasureNode::LegMeasureNode()
    : Node("leg_measure_node") {

    this->declare_parameter("joint1_rad",0.0);      //创建三个关节的期望位置参数
    this->declare_parameter("joint2_rad",0.0);
    this->declare_parameter("joint3_rad",-1.57);
    this->declare_parameter("leg_id",2);

    //订阅节点的位置信息
    legs_state_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_status", 10, std::bind(&LegMeasureNode::measureCallback, this, std::placeholders::_1));

        target_set_timer=this->create_wall_timer(100ms,[this](){
        robot_interfaces::msg::Robot leg_target;
            int leg_id=this->get_parameter("leg_id").as_int();
            leg_target.legs[leg_id].joints[0].rad=(float)this->get_parameter("joint1_rad").as_double();
            leg_target.legs[leg_id].joints[0].kp=0.1f;
            leg_target.legs[leg_id].joints[0].kd=0.2f;
            leg_target.legs[leg_id].joints[0].omega=0.0f;
            leg_target.legs[leg_id].joints[0].torque=0.0f;

            leg_target.legs[leg_id].joints[1].rad=(float)this->get_parameter("joint2_rad").as_double();
            leg_target.legs[leg_id].joints[1].kp=0.1f;
            leg_target.legs[leg_id].joints[1].kd=0.2f;
            leg_target.legs[leg_id].joints[1].omega=0.0f;
            leg_target.legs[leg_id].joints[1].torque=0.0f;

            leg_target.legs[leg_id].joints[2].rad=(float)this->get_parameter("joint3_rad").as_double();
            leg_target.legs[leg_id].joints[2].kp=0.1f;
            leg_target.legs[leg_id].joints[2].kd=0.2f;
            leg_target.legs[leg_id].joints[2].omega=0.0f;
            leg_target.legs[leg_id].joints[2].torque=0.0f;

            for(int i=0;i<4;i++)
            {
                if(leg_id!=i)
                {
                    leg_target.legs[i].joints[0].kp=0.0f;
                    leg_target.legs[i].joints[1].kp=0.0f;
                    leg_target.legs[i].joints[2].kp=0.0f;
                }
            }
            legs_target_pub->publish(leg_target);
        });
}

LegMeasureNode::~LegMeasureNode() {

}

void LegMeasureNode::measureCallback(const robot_interfaces::msg::Robot& msg) {
    // 处理接收到的腿部状态消息
    RCLCPP_INFO(this->get_logger(), "Received leg state message");
    // 在这里添加测量逻辑，例如记录数据到文件等
}

