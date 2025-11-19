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

    void setJointCurrentRad(const Vector3D &rad);           //设置关节当前的位置/角速度/力矩
    void setJointCurrentOmega(const Vector3D &omega);
    void setJointCurrentTorque(const Vector3D &torque);

    void setLegExptPos(const Vector3D&pos);                 //设置狗腿当前的位置/速度/力
    void setLegExpVel(const Vector3D &vel);
    void setLegExpAcc(const Vector3D &acc);
    void setLegExpForce(const Vector3D &force);
    
    Vector3D calculateExpJointRad(bool *arrivable);     //根据期望位置计算期望关节角度
    Vector3D calculateExpJointOmega();                  //根据期望角速度计算期望关节角速度
    Vector3D calculateExpJointTorque();                 //根据期望足端力矩计算关节期望力矩
    
    Vector3D calculateCurFootPosition();
    Vector3D calculateCurFootVelocity();
    Vector3D calculateCurFootForce();

    void MathReset();


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

    
private:
    bool exp_pos_is_update;
    bool exp_vel_is_update;
    bool exp_force_is_update;
    bool exp_acc_is_update;

    bool exp_rad_is_update;
    bool exp_omega_is_update;
    bool exp_torque_is_update;

    bool cur_rad_is_update;
    bool cur_omega_is_update;
    bool cur_torque_is_update;
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