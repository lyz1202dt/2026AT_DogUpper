#include "leg_driver.h"


LegDriver::LegDriver(void *joint1, void *joint2, void *joint3,bool inv_motor1,bool inv_motor2,bool inv_motor3)
{
    inverse_coefficient_m1=inv_motor1?-1:1;
    inverse_coefficient_m2=inv_motor2?-1:1;
    inverse_coefficient_m3=inv_motor3?-1:1;
}

void LegDriver::Set_Offset(float joint1_offset,float joint2_offset,float joint3_offset)
{
    this->joint1_offset=joint1_offset;
    this->joint2_offset=joint2_offset;
    this->joint3_offset=joint3_offset;
}

void LegDriver::Set_PD(float m1_kp,float m1_kd,float m2_kp,float m2_kd,float m3_kp,float m3_kd)
{
    joint1_kp=m1_kp;
    joint1_kd=m1_kd;
    joint2_kp=m2_kp;
    joint2_kd=m2_kd;
    joint3_kp=m3_kp;
    joint3_kd=m3_kd;
}

std::tuple<Eigen::Vector3d,Eigen::Vector3d,Eigen::Vector3d> LegDriver::SendAndReceiveMotor(const Eigen::Vector3d &rad,const Eigen::Vector3d &omega,const Eigen::Vector3d &tor)
{
    // GoMotorSend(joint1_handler, tor[0]/6.33*inverse_coefficient_m1, omega[0]*6.33*inverse_coefficient_m1, rad[0]*6.33*inverse_coefficient_m1+joint1_offset, joint1_kp, joint1_kd);     //转换为输出轴力矩，位置和速度
    // GoMotorRecv(joint1_handler);
    // GoMotorSend(joint2_handler, tor[1]/6.33*inverse_coefficient_m2, omega[1]*6.33*inverse_coefficient_m2, rad[1]*6.33*inverse_coefficient_m2+joint2_offset, joint2_kp, joint2_kp);
    // GoMotorRecv(joint2_handler);
    // GoMotorSend(joint3_handler, tor[2]/6.33*inverse_coefficient_m3, omega[2]*6.33*inverse_coefficient_m3, rad[2]*6.33*inverse_coefficient_m3+joint3_offset, joint3_kp, joint3_kp);
    // GoMotorRecv(joint3_handler);

    // Eigen::Vector3d cur_rad,cur_omega,cur_torque;           //标准化构造
    // cur_rad[0]=(joint1_handler->state.rad-joint1_offset)/6.33*inverse_coefficient_m1;
    // cur_omega[0]=(joint1_handler->state.velocity)/6.33*inverse_coefficient_m2;
    // cur_torque[0]=(joint1_handler->state.torque)*6.33*inverse_coefficient_m3;

    // cur_rad[1]=(joint2_handler->state.rad-joint2_offset)/6.33*inverse_coefficient_m1;
    // cur_omega[1]=(joint2_handler->state.velocity)/6.33*inverse_coefficient_m2;
    // cur_torque[1]=(joint2_handler->state.torque)*6.33*inverse_coefficient_m3;

    // cur_rad[2]=(joint3_handler->state.rad-joint3_offset)/6.33*inverse_coefficient_m1;
    // cur_omega[2]=(joint3_handler->state.velocity)/6.33*inverse_coefficient_m2;
    // cur_torque[2]=(joint3_handler->state.torque)*6.33*inverse_coefficient_m3;

    //return std::make_tuple(cur_rad,cur_omega,cur_torque);
}
