#include <chrono>
#include <mynode.h>

using namespace std::chrono_literals;

// 初始化狗腿参数，创建发布者
LegControl::LegControl(LegParam_t& leg_param, std::string name)
    : Node(name) {

    leg = new Leg(leg_param); // 创建数学解算对象

    marker_publisher =
        this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rviz_joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    legs_target_pub = this->create_publisher<robot_interfaces::msg::Robot>(
        "legs_target", 10); // 创建期望位置发布者

    legs_state_sub = this->create_subscription<robot_interfaces::msg::Robot>(
        "legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
            //RCLCPP_INFO(this->get_logger(), "订阅到最新的电机状态");
            leg->setJointCurrentState(
                Vector3D(msg.legs[2].joints[0].rad,msg.legs[2].joints[1].rad,msg.legs[2].joints[2].rad), 
                Vector3D(msg.legs[2].joints[0].omega,msg.legs[2].joints[1].omega,msg.legs[2].joints[2].omega),
                Vector3D(msg.legs[2].joints[0].torque,msg.legs[2].joints[1].torque,msg.legs[2].joints[2].torque));
        });

    leg_run_time = 2.0f;

    ui_update_timer = create_wall_timer(100ms, [this]() { Show_Cb(); });
    move_update_timer = create_wall_timer(20ms, [this]() { Run_Cb(); });
}

LegControl::~LegControl() { delete leg; }

void LegControl::Show_Cb(){

    auto cur_foot_pos=leg->calculateCurFootPosition();
    auto cur_foot_force=leg->calculateCurFootForce(leg->calculateMassComponentsTorque());

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name     = {"joint1", "joint2", "joint3"};
    joint_msg.position={leg->cur_joint_rad[0],leg->cur_joint_rad[1],leg->cur_joint_rad[2]};
    rviz_joint_publisher->publish(joint_msg);	//发布关节角度信息*/
    
    visualization_msgs::msg::Marker dot_marker;
    dot_marker.header.frame_id = "world"; // 设置坐标系
    dot_marker.header.stamp    = this->get_clock()->now();
    dot_marker.ns              = "points";
    dot_marker.id              = 0;
    dot_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    dot_marker.action          = visualization_msgs::msg::Marker::ADD;
    
    dot_marker.pose.position.x = cur_foot_pos[0];
    dot_marker.pose.position.y = cur_foot_pos[1];
    dot_marker.pose.position.z = cur_foot_pos[2];
    // 设置球体的尺寸
    dot_marker.scale.x = 0.1;
    dot_marker.scale.y = 0.1;
    dot_marker.scale.z = 0.1;
    // 设置颜色
    dot_marker.color.a = 1.0; // 不透明
    dot_marker.color.r = 1.0;
    dot_marker.color.g = 0.0;
    dot_marker.color.b = 0.0;

    marker_publisher->publish(dot_marker);      //发布点标记（狗腿足端位置）


    visualization_msgs::msg::Marker arraw_marker;
    arraw_marker.header.frame_id = "world";  // 选择你在 TF 树中有的 frame
    arraw_marker.header.stamp = this->get_clock()->now();
    arraw_marker.ns = "arrows";
    arraw_marker.id = 0;
    // 类型：箭头
    arraw_marker.type = visualization_msgs::msg::Marker::ARROW;
    arraw_marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p_start;
    p_start.x = cur_foot_pos[0];
    p_start.y = cur_foot_pos[1];
    p_start.z = cur_foot_pos[2];

    geometry_msgs::msg::Point p_end;
    p_end.x = p_start.x+cur_foot_force[0]*0.05f;
    p_end.y = p_start.y+cur_foot_force[1]*0.05f;
    p_end.z = p_start.z+cur_foot_force[2]*0.05f;

    arraw_marker.points.push_back(p_start);
    arraw_marker.points.push_back(p_end);

    arraw_marker.scale.x = 0.03;
    arraw_marker.scale.y = 0.03;
    arraw_marker.scale.z = 0.03;
    // 设置颜色
    arraw_marker.color.a = 1.0; // 不透明
    arraw_marker.color.r = 1.0;
    arraw_marker.color.g = 1.0;
    arraw_marker.color.b = 0.0;

    arraw_marker.lifetime = rclcpp::Duration(0, 0);

    marker_publisher->publish(arraw_marker);    //发布箭头标记（狗腿足端受力）*/
}

void LegControl::Run_Cb() {
    //计算当前时间下足端期望位置和速度
    /*if (cur_time == 0.0)      // 如果当前时间等于0,那么规划一次步态
    {
        step_update_flag=true;
        UpdateGndStepLine(leg->calculateCurFootPosition(), Vector2D(0.02, 0.0), &trajectory,leg_run_time);
    }
    else if(step_update_flag&&cur_time>=leg_run_time*0.5)   //如果支撑相走完，在摆动相开始时时再根据当前状态规划一次步态
    {
        step_update_flag=false;
        UpdateAirStepLine(leg->calculateCurFootPosition(), Vector3D(-0.02,0.0,0.0), Vector2D(0.02, 0.00), &trajectory,leg_run_time,0.1f);
    }

    auto leg_cart_target = GetLegTarget(cur_time, trajectory);

    leg->setLegExptPos(std::get<0>(leg_cart_target)); // 设置狗腿笛卡尔空间期望位置
    leg->setLegExpVel(std::get<1>(leg_cart_target));
    // TODO:设置期望速度/加速度，力矩等
    bool arrivable;
    auto leg_joint_target_rad = leg->calculateExpJointRad(&arrivable); // 计算狗腿关节空间位置
    auto leg_joint_target_omega=leg->calculateExpJointOmega(); //计算狗腿关节空间速度*/

    //计算当前时间下足端期望位置和速度
    auto now_time = std::chrono::high_resolution_clock::now();
    auto step_time=std::chrono::duration_cast<std::chrono::milliseconds>(now_time-last_step_reset_time);
    float cur_time = (float)step_time.count()/1000.0f;

    if (cur_time>leg_run_time)      //规划一次步态
    {
        last_step_reset_time=now_time;
        cur_time=0.0f;
        UpdateCycloidStep(Vector2D(0.2,0.07), &cycloid_step,leg_run_time,0.1f);
    }
    auto leg_cart_target  = GetCycloidStep(cur_time, cycloid_step); //笛卡尔系下的狗腿期望

    //RCLCPP_INFO(this->get_logger(),"足端实际位置:(%f,%f,%f)",leg->calculateCurFootPosition()[0],leg->calculateCurFootPosition()[1],leg->calculateCurFootPosition()[2]);

    bool arrivable;
    auto leg_joint_target_rad = leg->calculateExpJointRad(std::get<0>(leg_cart_target),&arrivable); // 计算狗腿关节空间位置
    auto leg_joint_target_omega=leg->calculateExpJointOmega(std::get<0>(leg_cart_target),std::get<1>(leg_cart_target)); //计算狗腿关节空间速度
    auto exp_joint_torque=leg->calculateMassComponentsTorque();

    //TODO:单腿VMC
    leg->calculateCurFootForce(exp_joint_torque);


    if (!arrivable) // 如果规划出来的轨迹是可到达的目标
    {
        RCLCPP_WARN(this->get_logger(), "足端期望位置不可达");
    }
    // 发布电机目标值
    robot_interfaces::msg::Robot joint_msg_driver;
    joint_msg_driver.legs[2].joints[0].rad = (float)leg_joint_target_rad[0];
    joint_msg_driver.legs[2].joints[1].rad = (float)leg_joint_target_rad[1];
    joint_msg_driver.legs[2].joints[2].rad = (float)leg_joint_target_rad[2];
    joint_msg_driver.legs[2].joints[0].omega = (float)leg_joint_target_omega[0];
    joint_msg_driver.legs[2].joints[1].omega = (float)leg_joint_target_omega[1];
    joint_msg_driver.legs[2].joints[2].omega = (float)leg_joint_target_omega[2];
    joint_msg_driver.legs[2].joints[0].torque = (float)exp_joint_torque[0];
    joint_msg_driver.legs[2].joints[1].torque = (float)exp_joint_torque[1];
    joint_msg_driver.legs[2].joints[2].torque = (float)exp_joint_torque[2];
    joint_msg_driver.legs[2].joints[0].kp = 1.5f;
    joint_msg_driver.legs[2].joints[1].kp = 0.19f;
    joint_msg_driver.legs[2].joints[2].kp = 0.2f;
    joint_msg_driver.legs[2].joints[0].kd = 0.1f;
    joint_msg_driver.legs[2].joints[1].kd = 0.05f;
    joint_msg_driver.legs[2].joints[2].kd = 0.05f;
    legs_target_pub->publish(joint_msg_driver);

    leg->MathReset(); // 本次控制周期已经结束，清除数学运算缓存信息
}
