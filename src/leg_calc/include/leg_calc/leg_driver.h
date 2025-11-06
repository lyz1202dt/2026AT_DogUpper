#ifndef __LEGDRIVER_H__
#define __LEGDRIVER_H__

#include <Eigen/Dense>

class LegDriver
{
public:
    LegDriver(void *joint1, void *joint2, void *joint3,bool inv_motor1=false,bool inv_motor2=false,bool inv_motor3=false); // 使用三个关节电机结构体初始化狗腿类
    void Set_Offset(float joint1_offset, float joint2_offset, float joint3_offset);
    void Set_PD(float m1_kp, float m1_kd, float m2_kp, float m2_kd, float m3_kp, float m3_kd);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> SendAndReceiveMotor(const Eigen::Vector3d &rad, const Eigen::Vector3d &omega, const Eigen::Vector3d &tor); // 发送三个电机的期望，并返回当前电机的扭矩,，位置，速度（阻塞函数）
private:
    float joint1_offset;
    float joint2_offset;
    float joint3_offset;
    float joint1_kp, joint1_kd;
    float joint2_kp, joint2_kd;
    float joint3_kp, joint3_kd;
    int inverse_coefficient_m1;
    int inverse_coefficient_m2;
    int inverse_coefficient_m3;
};

#endif
