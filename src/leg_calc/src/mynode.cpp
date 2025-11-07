#include "mynode.h"
#include <chrono>
#include "robot_interfaces/msg/robot.hpp"


using namespace std::chrono_literals;

//初始化狗腿参数，创建发布者
LegControl::LegControl(LegParam_t &leg_param,std::string name) :Node(name)
{
    legs_target_pub=this->create_publisher<robot_interfaces::msg::Robot>("legs_target",10);			//创建期望位置发布者
		// legs_state_sub should be a subscription, not a publisher
		legs_state_sub=this->create_subscription<robot_interfaces::msg::Robot>(
		    "legs_state", 10,
		    [this](const robot_interfaces::msg::Robot::SharedPtr &msg) {
				;
		    }
		);
	rviz_joint_publisher=this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

    time=2.01;
    update_flag=2;
    leg_run_time=2;

    leg=new Leg(leg_param);    //创建数学解算对象

	update_timer=create_wall_timer(10ms,[this](){Run_Cb();});
}

LegControl::~LegControl()
{
    delete leg;
}

void LegControl::Run_Cb()
{
    /*leg->SetMotorState(leg_state);

	//求解足端位置和速度
	std::get<0>(leg_target)=leg->CalculateFootPosition();
	std::get<1>(leg_target)=leg->CalculateFootVelocity();
		
	if(time>=leg_run_time&&update_flag==2)	//周期循环复位
	{
		update_flag=0;
		time=0.0f;
	}

	if((time>=leg_run_time*0.5)&&update_flag==1)	//需要规划抬腿动作
	{
		update_flag=2;
		UpdateStepLine(std::get<0>(leg_state),std::get<1>(leg_state),Eigen::Vector2d(0.05f,0.0f),&trajectory,leg_run_time,0.1f);
	}
	else if(time>=0.0f&&update_flag==0)	//需要规划平移动作
	{
		update_flag=1;
		UpdateStepLine(std::get<0>(leg_state),std::get<1>(leg_state),Eigen::Vector2d(0.05f,0.0f),&trajectory,leg_run_time,0.1f);
	}
	
	leg_target=GetLegTarget(time,trajectory);		//跟踪规划出来的曲线
	
	leg->SetLegTarget(	//设置狗腿期望的位置，速度，加速度，足端力矩
		std::get<0>(leg_target),
		Vector3D(0.0,0.0,0.0),
		Vector3D(0.0,0.0,0.0),
		Vector3D(0.0,0.0,0.0));
		
    bool arrivable=false;
	Vector3D temp=leg->CalculateJointPosition(&arrivable);	//根据期望的笛卡尔坐标系位置计算关节期望角度
	if(arrivable==true)
		std::get<0>(leg_target)=temp;
	
	//TODO:发布三个电机的三个期望
    leg_interfaces::msg::LegTarget msg;
    msg.joint1_rad=std::get<0>(leg_target)[0];
    msg.joint2_rad=std::get<0>(leg_target)[0];
    msg.joint3_rad=std::get<0>(leg_target)[0];
    msg.joint1_omega=std::get<0>(leg_target)[1];
    msg.joint2_omega=std::get<0>(leg_target)[1];
    msg.joint3_omega=std::get<0>(leg_target)[1];
    msg.joint1_torque=std::get<0>(leg_target)[2];
    msg.joint2_torque=std::get<0>(leg_target)[2];
    msg.joint3_torque=std::get<0>(leg_target)[2];
    target_publisher->publish(msg);

	sensor_msgs::msg::JointState joint_msg;
	joint_msg.header.stamp=this->get_clock()->now();

	joint_msg.name={"joint1","joint2","joint3"};
	joint_msg.position = {
        std::get<0>(leg_target)[0],
        std::get<0>(leg_target)[1],
        std::get<0>(leg_target)[2]
    };

	rviz_joint_publisher->publish(joint_msg);

    leg_state=leg_target;   //模拟狗腿位置
	
	leg->MathReset();			//已经完成一个控制周期，清除数学计算缓存信息
    time=time+0.001;*/
    leg->SetLegTarget(	//设置狗腿期望的位置，速度，加速度，足端力矩
		Vector3D(0.2+time/8.0,0.0,0.0),
		Vector3D(0.0,0.0,0.0),
		Vector3D(0.0,0.0,0.0),
		Vector3D(0.0,0.0,0.0));

    bool arrivable=false;
	Vector3D temp=leg->CalculateJointPosition(&arrivable);	//根据期望的笛卡尔坐标系位置计算关节期望角度
	if(arrivable==true)
		std::get<0>(leg_target)=temp;

    sensor_msgs::msg::JointState joint_msg;
	joint_msg.header.stamp=this->get_clock()->now();

	joint_msg.name={"joint1","joint2","joint3"};
	joint_msg.position = {
        std::get<0>(leg_target)[0],
        std::get<0>(leg_target)[1],
        std::get<0>(leg_target)[2]
    };
    if(arrivable)
        RCLCPP_INFO(get_logger(),"可以到达:joint1=%lf,joint2=%lf,joint3=%lf",std::get<0>(leg_target)[0],std::get<0>(leg_target)[1],std::get<0>(leg_target)[2]);
    else
        RCLCPP_INFO(get_logger(),"不可到达");

	rviz_joint_publisher->publish(joint_msg);
    
    leg->MathReset();
	time=time>leg_run_time?0.0:(time+0.01);
}
