#include <chrono>
#include <cmath>
#include <mynode.h>

using namespace std::chrono_literals;

// 初始化狗腿参数，创建发布者
LegControl::LegControl(LegParam_t& leg_param, std::string name)
    : Node(name) {

    force_filter_gate=0.8;
    foot_force=Vector3D(0.0,0.0,0.0);

    leg = new Leg(leg_param); // 创建数学解算对象
    vmc = new VMC(200,60,5.0,0.5,0.2,0.1,20ms);   //创建VMC计算对象

    this->declare_parameter("joint1_kp",2.4);
    this->declare_parameter("joint2_kp",3.2);
    this->declare_parameter("joint3_kp",1.5);
    this->declare_parameter("joint1_kd",0.15);
    this->declare_parameter("joint2_kd",0.16);
    this->declare_parameter("joint3_kd",0.18);
    this->declare_parameter("force_filter_gate",0.8);
    this->declare_parameter("enable_vmc",false);
    this->declare_parameter("vmc_kp",350.0);
    this->declare_parameter("vmc_kd",50.0);
    this->declare_parameter("vmc_mass",5.0);
    
    param_server_handle=this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(this->get_logger(),"更新参数",(int)params.size());
        for(const auto &param:params)
        {
            if(param.get_name()=="enable_vmc")
                enable_vmc=param.as_bool();
            else if(param.get_name()=="joint1_kp")
                joint1_kp=param.as_double();
            else if(param.get_name()=="joint1_kd")
                joint1_kd=param.as_double();
            else if(param.get_name()=="joint2_kp")
                joint2_kp=param.as_double();
            else if(param.get_name()=="joint2_kd")
                joint2_kd=param.as_double();
            else if(param.get_name()=="joint3_kp")
                joint3_kp=param.as_double();
            else if(param.get_name()=="joint3_kd")
                joint3_kd=param.as_double();
            else if(param.get_name()=="force_filter_gate")
                force_filter_gate=param.as_double();
            else if(param.get_name()=="vmc_kp")
                vmc->kp=param.as_double();
            else if(param.get_name()=="vmc_kd")
                vmc->kd=param.as_double();
            else if(param.get_name()=="vmc_mass")
                vmc->mass=param.as_double();
        }
        return result;
    });

    joint1_kp=get_parameter("joint1_kp").as_double();
    joint1_kd=get_parameter("joint1_kd").as_double();
    joint2_kp=get_parameter("joint2_kp").as_double();
    joint2_kd=get_parameter("joint2_kd").as_double();
    joint3_kp=get_parameter("joint3_kp").as_double();
    joint3_kd=get_parameter("joint3_kd").as_double();

    

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

LegControl::~LegControl() {
    delete leg;
    delete vmc;
}

void LegControl::Show_Cb(){

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name     = {"joint1", "joint2", "joint3"};
    joint_msg.position={leg->cur_joint_rad[0],leg->cur_joint_rad[1],leg->cur_joint_rad[2]};
    rviz_joint_publisher->publish(joint_msg);	//发布关节角度信息
    
    visualization_msgs::msg::Marker dot_marker;
    dot_marker.header.frame_id = "world"; // 设置坐标系
    dot_marker.header.stamp    = this->get_clock()->now();
    dot_marker.ns              = "points";
    dot_marker.id              = 0;
    dot_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    dot_marker.action          = visualization_msgs::msg::Marker::ADD;
    
    dot_marker.pose.position.x = foot_pos[0];
    dot_marker.pose.position.y = foot_pos[1];
    dot_marker.pose.position.z = foot_pos[2];
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
    p_start.x = foot_pos[0];
    p_start.y = foot_pos[1];
    p_start.z = foot_pos[2];

    geometry_msgs::msg::Point p_end;
    //p_end.x = p_start.x+foot_force[0]*0.05f;
    //p_end.y = p_start.y+foot_force[1]*0.05f;
    //p_end.z = p_start.z+foot_force[2]*0.05f;
    p_end.x = p_start.x+foot_force[0]*0.05;
    p_end.y = p_start.y+foot_force[1]*0.05;
    p_end.z = p_start.z+leg_virtual_force*0.05;

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

    RCLCPP_INFO(get_logger(),"kp=%lf,kd=%lf,mass=%lf",vmc->kp,vmc->kd,vmc->mass);
}

void LegControl::Run_Cb() {

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
    //重力补偿计算
    auto grivate_compen_torque=leg->calculateMassComponentsTorque();
    //TODO:单腿VMC
    foot_pos=leg->calculateCurFootPosition();
    foot_vel=leg->calculateCurFootVelocity();
    auto raw_foot_force=leg->calculateCurFootForce(grivate_compen_torque);
    RCLCPP_INFO(get_logger(),"足端原始受力%lf",raw_foot_force[2]);

    if(raw_foot_force[2]>-50.0&&raw_foot_force[2]<50.0)         //排除计算异常的情况
        foot_force[2]=0.8*foot_force[2]+0.2*raw_foot_force[2];
    std::tuple<double,double,double> vmc_target=std::make_tuple(0.0,0.0,0.0);
    if(enable_vmc)     //使用VMC
    {
        vmc_target=vmc->targetUpdate(0.0,foot_pos[2],0.0, foot_vel[2], -foot_force[2]);
        leg_virtual_force=std::get<2>(vmc_target);
        RCLCPP_INFO(this->get_logger(),"vmc(%lf,%lf,%lf)",std::get<0>(vmc_target),std::get<1>(vmc_target),std::get<2>(vmc_target));
    }

    //计算足端位置
    bool arrivable=false;
    auto leg_joint_target_rad = leg->calculateExpJointRad(Vector3D(0.0,0.0,std::get<0>(vmc_target)),&arrivable); // 计算狗腿关节空间位置
    auto leg_joint_target_omega=leg->calculateExpJointOmega(Vector3D(0.0,0.0,std::get<0>(vmc_target)),Vector3D(0.0,0.0,std::get<1>(vmc_target))); //计算狗腿关节空间速度
    //auto leg_vmc_torque=leg->calculateFootForceTorque(Vector3D(0.0,0.0,std::get<0>(vmc_target)), Vector3D(0.0,0.0,std::get<2>(vmc_target)));
    RCLCPP_INFO(this->get_logger(),"足端受力(%lf,%lf,%lf)",foot_force[0],foot_force[1],foot_force[2]);
    RCLCPP_INFO(this->get_logger(),"足端位置(%lf,%lf,%lf)",foot_pos[0],foot_pos[1],foot_pos[2]);

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
    joint_msg_driver.legs[2].joints[0].torque = (float)grivate_compen_torque[0];//-leg_vmc_torque[0];
    joint_msg_driver.legs[2].joints[1].torque = (float)grivate_compen_torque[1];//-leg_vmc_torque[1];
    joint_msg_driver.legs[2].joints[2].torque = (float)grivate_compen_torque[2];//-leg_vmc_torque[2];
    joint_msg_driver.legs[2].joints[0].kp = (float)this->get_parameter("joint1_kp").as_double();
    joint_msg_driver.legs[2].joints[1].kp = (float)this->get_parameter("joint2_kp").as_double();
    joint_msg_driver.legs[2].joints[2].kp = (float)this->get_parameter("joint3_kp").as_double();
    joint_msg_driver.legs[2].joints[0].kd = (float)this->get_parameter("joint1_kd").as_double();
    joint_msg_driver.legs[2].joints[1].kd = (float)this->get_parameter("joint2_kd").as_double();
    joint_msg_driver.legs[2].joints[2].kd = (float)this->get_parameter("joint3_kd").as_double();
    legs_target_pub->publish(joint_msg_driver);

    leg->MathReset(); // 本次控制周期已经结束，清除数学运算缓存信息
}
