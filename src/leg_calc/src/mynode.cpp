#include <mynode.h>

using namespace std::chrono_literals;

// 初始化狗腿参数，创建发布者
LegControl::LegControl(LegParam_t& leg_param, std::string name)
    : Node(name) {

    marker_publisher =
        this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rviz_joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = this->create_publisher<robot_interfaces::msg::Robot>(
        "legs_target", 10); // 创建期望位置发布者

    legs_state_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
            //RCLCPP_INFO(this->get_logger(), "订阅到最新的电机状态");
            /*sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name     = {"joint1", "joint2", "joint3"};
            joint_msg.position={msg.legs[2].joints[0].rad,msg.legs[2].joints[1].rad,msg.legs[2].joints[2].rad};
            rviz_joint_publisher->publish(joint_msg);	//发布关节角度信息*/

            leg->setJointCurrentRad(Vector3D(
                msg.legs[2].joints[0].rad, msg.legs[2].joints[1].rad, msg.legs[2].joints[2].rad));
            leg->setJointCurrentOmega(Vector3D(
                msg.legs[2].joints[0].omega, msg.legs[2].joints[1].omega,
                msg.legs[2].joints[2].omega));
            leg->setJointCurrentTorque(Vector3D(
                msg.legs[2].joints[0].torque, msg.legs[2].joints[1].torque,
                msg.legs[2].joints[2].torque));
        });

    cur_time     = 0;
    state_flag   = 0;
    leg_run_time = 2.0;
    step_update_flag=true;

    leg = new Leg(leg_param); // 创建数学解算对象

    update_timer = create_wall_timer(200ms, [this]() { Run_Cb(); });
}

LegControl::~LegControl() { delete leg; }

void LegControl::Run_Cb() {

    if (cur_time == 0.0)      // 如果当前时间等于0,那么规划一次步态
    {
        step_update_flag=true;
        UpdateGndStepLine(Vector3D(0.05,0.00,0.0), Vector2D(0.1, 0.05), &trajectory,leg_run_time);
    }
    else if(step_update_flag&&cur_time>=leg_run_time*0.5)   //如果支撑相走完，在摆动相开始时时再根据当前状态规划一次步态
    {
        step_update_flag=false;
        UpdateAirStepLine(Vector3D(-0.05,0.00,0.0), Vector3D(-0.05,0.0,0.0), Vector2D(0.1, 0.05), &trajectory,leg_run_time,0.1f);
    }

    auto leg_cart_target = GetLegTarget(cur_time, trajectory);

    leg->setLegExptPos(std::get<0>(leg_cart_target)); // 设置狗腿笛卡尔空间期望位置
    // TODO:设置期望速度/加速度，力矩等
    bool arrivable;
    auto leg_joint_target = leg->calculateExpJointRad(&arrivable); // 计算狗腿关节空间位置
    if (!arrivable) // 如果规划出来的轨迹是可到达的目标
    {
        RCLCPP_WARN(this->get_logger(), "足端期望位置不可达");
    }
    RCLCPP_INFO(this->get_logger(),"足端位置:(%f,%f,%f)",std::get<0>(leg_cart_target)[0],std::get<0>(leg_cart_target)[1],std::get<0>(leg_cart_target)[2]);
    
    // 发布标记表示期望的足端位置
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "link0"; // 设置坐标系
    marker.header.stamp    = this->get_clock()->now();
    marker.ns              = "points";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::SPHERE;
    marker.action          = visualization_msgs::msg::Marker::ADD;

    // 设置位置
    //marker.pose.position.x = std::get<0>(leg_cart_target)[0];
    //marker.pose.position.y = std::get<0>(leg_cart_target)[1];
    //marker.pose.position.z = std::get<0>(leg_cart_target)[2];

    //auto leg_cur_cart_posleg->calculateCurFootPosition();
    
    marker.pose.position.x = leg->exp_cart_pos[0];
    marker.pose.position.y = leg->exp_cart_pos[1];
    marker.pose.position.z = leg->exp_cart_pos[2];
    //RCLCPP_INFO(this->get_logger(),"关节期望角度%f,%f,%f",leg_joint_target[0],leg_joint_target[1],leg_joint_target[2]);
    //RCLCPP_INFO(this->get_logger(),"关节实际角度%f,%f,%f",leg->cur_joint_pos[0],leg->cur_joint_pos[1],leg->cur_joint_pos[2]);

    // 设置球体的尺寸
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // 设置颜色
    marker.color.a = 1.0; // 不透明
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 发布marker
    marker_publisher->publish(marker);

    // 发布rviz2可视化消息
    sensor_msgs::msg::JointState joint_msg_rviz2;
    joint_msg_rviz2.header.stamp = this->get_clock()->now();
    joint_msg_rviz2.name         = {"joint1", "joint2", "joint3"};
    joint_msg_rviz2.position     = {leg->exp_joint_pos[0], leg->exp_joint_pos[1], leg->exp_joint_pos[2]};
    rviz_joint_publisher->publish(joint_msg_rviz2);

    // 发布电机目标值
    robot_interfaces::msg::Robot joint_msg_driver;
    joint_msg_driver.legs[2].joints[0].rad = (float)leg_joint_target[0];
    joint_msg_driver.legs[2].joints[1].rad = (float)leg_joint_target[1];
    joint_msg_driver.legs[2].joints[2].rad = (float)leg_joint_target[2];
    // TODO:写入每个关节的Kp/Kd
    // legs_target_pub->publish(joint_msg_driver);

    leg->MathReset(); // 本次控制周期已经结束，清除数学运算缓存信息
    cur_time = cur_time > leg_run_time ? 0.0 : (cur_time + 0.04);
}
