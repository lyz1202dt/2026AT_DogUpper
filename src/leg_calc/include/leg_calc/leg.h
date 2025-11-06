#ifndef __LEG_H__
#define __LEG_H__

#include <cmath>
#include <Eigen/Dense>
#include <tuple>


typedef Eigen::Vector3d Vector3D;

typedef struct{
    double l0;              //电机安装槽到基旋转关节的位移（沿Joint1的z轴方向）
    double l1;              //电机1旋转平面到电机2旋转中心的距离
    double d2;              //关节3到关节2的z轴距离
    double l2;              //关节3到关节2的x轴距离
    double l3;              //足端到关节3的距离

    double m1;              //连杆质量
    double m2;
    double m3;
    Eigen::Matrix3d I1;     //连杆惯性张量
    Eigen::Matrix3d I2;
    Eigen::Matrix3d I3;
    Vector3D r1;            //质心坐标
    Vector3D r2;
    Vector3D r3;

    Eigen::Matrix4d T_GndToBase;    //狗腿零相位到机械臂基坐标的齐次变换矩阵
    Eigen::Matrix3d R_GndToBase;    //狗腿零相位到机械臂基坐标的齐次变换矩阵
    Eigen::Matrix3d R_1to0, R_2to1, R_3to2;  //各关节坐标系之间的旋转矩阵
}LegParam_t;


class Leg{
    public:

    static constexpr char FINISH_EXP_POS_CALC=0x01;
    static constexpr char FINISH_EXP_VEL_CALC=0x02;
    static constexpr char FINISH_EXP_TOR_CALC=0x04;

    static constexpr char FINISH_CUR_POS_CALC=0x10;
    static constexpr char FINISH_CUR_VEL_CALC=0x20;
    static constexpr char FINISH_CUR_TOR_CALC=0x40;
    
    Leg(LegParam_t &arg):param(arg) {}
    void SetMotorState(const std::tuple<Vector3D,Vector3D,Vector3D> &state);
    void SetMotorState(const Vector3D &angle,const Vector3D &omega,const Vector3D &torque);      //根据硬件读取到的数据设置该腿3个关节电机当前的角度，角速度，力矩
    void SetLegTarget(const Vector3D &pos,const Vector3D &vel,const Vector3D &acc,const Vector3D &F);   //设置狗腿的期望
    Vector3D CalculateJointPosition(bool *arrivable);   //计算关节位置，并存储关节空间期望角度
    Vector3D CalculateJointOmega(void);                 //计算关节角速度，并存储对位置求导的雅可比矩阵
    Vector3D CalculateJointTorque(void);                //计算关节力矩（求出角加速度后，使用牛顿欧拉方程递推）
    Vector3D CalculateFootForce(void);                  //观测足端受力，用于VMC控制
    Vector3D CalculateFootVelocity(void);               //观测足端速度，用于VMC控制
    Vector3D CalculateFootPosition(void);               //观测足端位置，用于VMC控制

    void MathReset(void);
private:
    char param_calculated;

    LegParam_t param;               //狗腿机械结构参数

    Vector3D cur_joint_pos;         //关节空间实际
    Vector3D cur_joint_vel;
    Vector3D cur_joint_tor;

    Vector3D cur_cart_pos;      //笛卡尔坐标系下的位置，速度和受力
    Vector3D cur_cart_vel;
    Vector3D cur_cart_for;

    Vector3D exp_joint_pos;         //关节空间期望
    Vector3D exp_joint_vel;
    Vector3D exp_joint_tor;

    Vector3D exp_cart_pos;
    Vector3D exp_cart_vel;
    Vector3D exp_cart_acc;
    Vector3D exp_cart_for;

    Eigen::Matrix3d jocabain_exp_pos;   //期望位置映射的雅可比矩阵
    Eigen::Matrix3d jocabain_cur_pos;   //实际位置映射的雅可比矩阵
};


/*
齐次变换矩阵：

>> A
 
A =
 
[1, 0, 0,  0]
[0, 1, 0,  0]
[0, 0, 1, l0]
[0, 0, 0,  1]
 
>> B
 
B =
 
[cos(q1), -sin(q1), 0, 0]
[sin(q1),  cos(q1), 0, 0]
[      0,        0, 1, 0]
[      0,        0, 0, 1]
 
>> C
 
C =
 
[1, 0, 0,  0]
[0, 1, 0,  0]
[0, 0, 1, l1]
[0, 0, 0,  1]
 
>> D

D =

     0    -1     0     0
     0     0    -1     0
     1     0     0     0
     0     0     0     1

>> E
 
E =
 
[cos(q2), -sin(q2), 0, 0]
[sin(q2),  cos(q2), 0, 0]
[      0,        0, 1, 0]
[      0,        0, 0, 1]
 
>> F
 
F =
 
[1, 0, 0, l2]
[0, 1, 0,  0]
[0, 0, 1, d2]
[0, 0, 0,  1]
 
>> G
 
G =
 
[cos(q3), -sin(q3), 0, 0]
[sin(q3),  cos(q3), 0, 0]
[      0,        0, 1, 0]
[      0,        0, 0, 1]
 
>> H
 
H =
 
[1, 0, 0, l3]
[0, 1, 0,  0]
[0, 0, 1,  0]
[0, 0, 0,  1]
 
>> I

I =

     0
     0
     0
     1

*/


#endif