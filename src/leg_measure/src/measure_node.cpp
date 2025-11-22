#include "measure_node.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

LegMeasureNode::LegMeasureNode()
    : Node("leg_measure_node") {

        start_measure=false;
        is_sampling=false;

    this->declare_parameter("joint1_rad",0.0);      //创建三个关节的期望位置参数
    this->declare_parameter("joint2_rad",0.0);
    this->declare_parameter("joint3_rad",-1.57);
    this->declare_parameter("leg_id",2);
    this->declare_parameter("csv_file_path","/home/lyz/Project/26_AT_dog/src/launch_pack/src/CSV.csv");
    this->declare_parameter("enable_measure",false);

    //订阅节点的位置信息
    legs_state_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_status", 10, std::bind(&LegMeasureNode::measureCallback, this, std::placeholders::_1));

        legs_target_pub=this->create_publisher<robot_interfaces::msg::Robot>("legs_target",10);

        target_set_timer=this->create_wall_timer(100ms,[this](){
        robot_interfaces::msg::Robot leg_target;
            int leg_id=this->get_parameter("leg_id").as_int();
            leg_target.legs[leg_id].joints[0].rad=(float)this->get_parameter("joint1_rad").as_double();
            leg_target.legs[leg_id].joints[0].kp=1.5f;
            leg_target.legs[leg_id].joints[0].kd=0.1f;
            leg_target.legs[leg_id].joints[0].omega=0.0f;
            leg_target.legs[leg_id].joints[0].torque=0.0f;

            leg_target.legs[leg_id].joints[1].rad=(float)this->get_parameter("joint2_rad").as_double();
            leg_target.legs[leg_id].joints[1].kp=0.19f;
            leg_target.legs[leg_id].joints[1].kd=0.05f;
            leg_target.legs[leg_id].joints[1].omega=0.0f;
            leg_target.legs[leg_id].joints[1].torque=0.0f;

            leg_target.legs[leg_id].joints[2].rad=(float)this->get_parameter("joint3_rad").as_double();
            leg_target.legs[leg_id].joints[2].kp=0.2f;
            leg_target.legs[leg_id].joints[2].kd=0.05f;
            leg_target.legs[leg_id].joints[2].omega=0.0f;
            leg_target.legs[leg_id].joints[2].torque=0.0f;
            for(int i=0;i<4;i++)        //其它的狗腿设置期望均为0
            {
                if(leg_id!=i)
                {
                    leg_target.legs[i].joints[0].kp=0.0f;
                    leg_target.legs[i].joints[1].kp=0.0f;
                    leg_target.legs[i].joints[2].kp=0.0f;
                    leg_target.legs[i].joints[0].kd=0.0f;
                    leg_target.legs[i].joints[1].kd=0.0f;
                    leg_target.legs[i].joints[2].kd=0.0f;
                }
            }
            RCLCPP_INFO(get_logger(),"发布腿关节目标位置");
            legs_target_pub->publish(leg_target);
        });
}


/*按照以下格式追加存储测量的数据:
joint1_rad,joint1_torque
joint2_rad,joint2_torque
joint3_rad,joint3_torque
*/

bool LegMeasureNode::saveResultToFile()
{
    if(is_sampling)
        return false;
    if(!start_measure)
        return false;
    int size=rad_torque_sample.size();
    double average[3][2]={};
    for(int i=0;i<size;i++)
    {
        average[0][0]+=rad_torque_sample[i][0];
        average[0][1]+=rad_torque_sample[i][1];
        average[1][0]+=rad_torque_sample[i][2];
        average[1][1]+=rad_torque_sample[i][3];
        average[2][0]+=rad_torque_sample[i][4];
        average[2][1]+=rad_torque_sample[i][5];
    }

    for(int i=0;i<3;i++)    //求u平均值
    {
        average[i][0]/=size;
        average[i][1]/=size;
    }

    std::ofstream file(this->get_parameter("csv_file_path").as_string(),std::ios::app);
        file << average[0][0] <<" , "<< average[0][1] << "\n"
            <<  average[1][0] <<" , "<< average[1][1] << "\n"
            <<  average[2][0] <<" , "<< average[2][1] << "\n";

    file.close();
    RCLCPP_INFO(this->get_logger(),"保存到CSV文件成功");
    return true;
}

void LegMeasureNode::startMeasure(int sample_num)
{
    rad_torque_sample.resize((unsigned long int)sample_num);    //修改vector容量为目标采样点的数量
}

LegMeasureNode::~LegMeasureNode() {
    
}

void LegMeasureNode::measureCallback(const robot_interfaces::msg::Robot& msg) {
    // 处理接收到的腿部状态消息
    RCLCPP_INFO(this->get_logger(), "接收到狗腿的信息");
    bool enable_measure=get_parameter("enable_measure").as_bool();
    if(enable_measure&&(!start_measure))
    {
        RCLCPP_INFO(get_logger(),"开始测量数据对");
        
        startMeasure(200);
        is_sampling=true;
        start_measure=true;
        cur_index=0;
    }


    if(is_sampling)
    {
        int leg_id=this->get_parameter("leg_id").as_int();
        rad_torque_sample[cur_index][0]=msg.legs[leg_id].joints[0].rad;
        rad_torque_sample[cur_index][1]=msg.legs[leg_id].joints[0].torque;
        rad_torque_sample[cur_index][2]=msg.legs[leg_id].joints[1].rad;
        rad_torque_sample[cur_index][3]=msg.legs[leg_id].joints[1].torque;
        rad_torque_sample[cur_index][4]=msg.legs[leg_id].joints[2].rad;
        rad_torque_sample[cur_index][5]=msg.legs[leg_id].joints[2].torque;
        RCLCPP_INFO(get_logger(),"采样中%d/%d",cur_index+1,(int)rad_torque_sample.size());
        cur_index++;
        if(cur_index==rad_torque_sample.size())
        {
            cur_index=0;
            is_sampling=false;
            RCLCPP_INFO(this->get_logger(),"采样完成");
        }
    }

    if(start_measure&&(!is_sampling))   //已经开始测量并且已经停止记采样
    {
        RCLCPP_INFO(get_logger(),"保存测量结果到CSV文件");
        saveResultToFile();
        start_measure=false;
        set_parameter(rclcpp::Parameter("enable_measure",false));   //将测量使能参数关闭
    }

}

