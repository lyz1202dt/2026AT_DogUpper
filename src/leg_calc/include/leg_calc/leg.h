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

    Eigen::Matrix<double, 6, 1> grivate_param;
}LegParam_t;


class Leg{
    public:
    
    Leg(LegParam_t &arg):param(arg) {}

    void setJointCurrentState(const Vector3D &rad,const Vector3D &omega,const Vector3D &torque);           //设置关节当前的位置/角速度/力矩
    
    Vector3D calculateExpJointRad(const Vector3D &cart_exp_pos,bool *arrivable);     //计算期望位置
    Vector3D calculateExpJointOmega(const Vector3D &cart_exp_pos, const Vector3D &cart_exp_vel);       //计算期望角速度
    Vector3D calculateAccelerationTorque(const Vector3D &cart_exp_pos, const Vector3D &vel, const Vector3D &acc);     //根据加速度计算力矩
    Vector3D calculateFootForceTorque(const Vector3D &cart_exp_pos,const Vector3D &cart_force);  //根据足端期望力计算关节力矩
    Vector3D calculateMassComponentsTorque();                        //计算狗腿重力补偿力矩
    
    Vector3D calculateCurFootPosition();                                   //计算足端的实际位置
    Vector3D calculateCurFootVelocity();   //计算足端的速度
    Vector3D calculateCurFootForce(const Vector3D &feedforward);          //除去feedforward前馈力矩后，计算实际足端受力

    void MathReset();


    LegParam_t param;               //狗腿机械结构参数

    Vector3D exp_joint_rad;         //关节空间期望位置
    Vector3D exp_joint_omega;       //关节空间期望速度
    Vector3D cur_joint_rad;         //狗腿当前的状态
    Vector3D cur_joint_omega;
    Vector3D cur_joint_torque;
    Vector3D cur_cart_pos;          //笛卡尔空间下足端位置
    Eigen::Matrix3d jocabain_exp_pos;   //期望位置映射的雅可比矩阵
    Eigen::Matrix3d jocabain_cur_pos;   //实际位置映射的雅可比矩阵
private:
    bool exp_jacobian_is_update{false};
    bool cur_jacobian_is_update{false};
    bool exp_omega_is_update{false};
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