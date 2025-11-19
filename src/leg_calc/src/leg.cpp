#include "leg.h"

void Leg::setJointCurrentRad(const Vector3D &rad)
{
    cur_joint_pos=rad;
    
    double s1 = std::sin(cur_joint_pos[0]);
    double c1 = std::cos(cur_joint_pos[0]);
    double s2 = std::sin(cur_joint_pos[1]);
    double c2 = std::cos(cur_joint_pos[1]);
    double s3 = std::sin(cur_joint_pos[2]);
    double c3 = std::cos(cur_joint_pos[2]);

    double l0=param.l0;
    double d2 = param.d2; // 机械臂参数:长度，质心长度，质量
    double l2 = param.l2;
    double l3 = param.l3;
    double l1 = param.l1;

    //clang-format off
    cur_cart_pos <<     //计算足端在笛卡尔坐标系中的位置
    d2*s1 - l3*(c1*c2*s3 + c1*c3*s2) - l2*c1*s2,
    -l3*(c2*s1*s3 + c3*s1*s2) - d2*c1 - l2*s1*s2,
    l0 + l1 + l3*(c2*c3 - s2*s3) + l2*c2;

    jocabain_cur_pos <<     //计算雅可比矩阵，准备用来求速度
    l3*(c2*s1*s3 + c3*s1*s2) + d2*c1 + l2*s1*s2, - l3*(c1*c2*c3 - c1*s2*s3) - l2*c1*c2  , -l3*(c1*c2*c3 - c1*s2*s3),
    d2*s1 - l3*(c1*c2*s3 + c1*c3*s2) - l2*c1*s2,   l3*(s1*s2*s3 - c2*c3*s1) - l2*c2*s1  ,  l3*(s1*s2*s3 - c2*c3*s1),
    0.0                                        ,  - l3*(c2*s3 + c3*s2) - l2*s2          , -l3*(c2*s3 + c3*s2);
    //clangd-format on

    cur_rad_is_update=true;
}

void Leg::setJointCurrentOmega(const Vector3D &omega)
{
    cur_joint_vel=omega;
    cur_omega_is_update=true;
}

void Leg::setJointCurrentTorque(const Vector3D &torque)
{
    cur_joint_tor=torque;
    cur_torque_is_update=true;
}

void Leg::setLegExptPos(const Vector3D &pos)           //设置关节当前的位置/角速度/力矩
{
    Eigen::Vector4d temp=param.T_GndToBase.inverse()*pos.homogeneous();
    exp_cart_pos=Vector3D(temp[0],temp[1],temp[2]);
    exp_pos_is_update=true;
}
void Leg::setLegExpVel(const Vector3D &vel)
{
    exp_cart_vel=param.R_GndToBase.transpose()*vel;
    exp_vel_is_update=true;
}

void Leg::setLegExpAcc(const Vector3D &acc)
{
    exp_cart_acc=param.R_GndToBase.transpose()*acc;
    exp_vel_is_update=true;
}
    
void Leg::setLegExpForce(const Vector3D &force)
{
    exp_cart_for=param.R_GndToBase.transpose()*force;
    exp_force_is_update=true;
}

void Leg::MathReset()
{
    exp_pos_is_update=false;
    exp_vel_is_update=false;
    exp_force_is_update=false;
    exp_acc_is_update=false;

    exp_rad_is_update=false;
    exp_omega_is_update=false;
    exp_torque_is_update=false;

    cur_rad_is_update=false;
    cur_omega_is_update=false;
    cur_torque_is_update=false;
}

Vector3D Leg::calculateExpJointRad(bool *arrivable)
{
    double dis2 = exp_cart_pos[0] * exp_cart_pos[0] + exp_cart_pos[1] * exp_cart_pos[1] - param.d2 * param.d2;
    if (dis2 < 0)
    {
        *arrivable=false;
        return cur_joint_pos;
    }
    
    double a = std::sqrt(dis2);
    double b = exp_cart_pos[2];
    
    double temp=(a * a + b * b - param.l2 * param.l2 - param.l3 * param.l3) / (2.0 * param.l2 * param.l3);
    if(temp>=1.0f||temp<=-1.0f)
    {
        *arrivable=false;
        return cur_joint_pos;
    }
    
    double q1=std::atan2(exp_cart_pos[1], exp_cart_pos[0]) - std::asin(param.d2/std::sqrt(exp_cart_pos[0] * exp_cart_pos[0] + exp_cart_pos[1] * exp_cart_pos[1])); // 末端目标点只会在x大于0的象限，并且由于角度限制不会到另一侧
    double q3 = std::acos(temp);
    if (q3 > 0.0) // 因为机械结构角度限制，只有q3角度小于0时，才是可达的
        q3 = -q3;
    double q2 = std::atan2(b, a) - std::atan2(param.l3 * std::sin(q3), param.l2 + param.l3 * std::cos(q3)); // 计算得到q2

    exp_joint_pos<<q1,(q2-1.570796325),q3;

    *arrivable=true;

    exp_rad_is_update=true;

    return exp_joint_pos;
}

Vector3D Leg::calculateExpJointOmega()
{
    if(!exp_rad_is_update)     //如果之前没有计算过位置，那么计算一次关节期望位置
    {
        bool arrivalbe=false;
        calculateExpJointRad(&arrivalbe);
        if(!arrivalbe)
            return Vector3D(0.0,0.0,0.0);
    }

    if(exp_omega_is_update)    //之前被调用过，那么立即返回
        return exp_joint_vel;

    double s1 = std::sin(exp_joint_pos[0]);
    double c1 = std::cos(exp_joint_pos[0]);
    double s2 = std::sin(exp_joint_pos[1]);
    double c2 = std::cos(exp_joint_pos[1]);
    double s3 = std::sin(exp_joint_pos[2]);
    double c3 = std::cos(exp_joint_pos[2]);

    double l0=param.l0;
    double d2 = param.d2; // 机械臂参数:长度，质心长度，质量
    double l2 = param.l2;
    double l3 = param.l3;
    double l1 = param.l1;

    jocabain_exp_pos <<
    l3*(c2*s1*s3 + c3*s1*s2) + d2*c1 + l2*s1*s2, - l3*(c1*c2*c3 - c1*s2*s3) - l2*c1*c2  , -l3*(c1*c2*c3 - c1*s2*s3),
    d2*s1 - l3*(c1*c2*s3 + c1*c3*s2) - l2*c1*s2,   l3*(s1*s2*s3 - c2*c3*s1) - l2*c2*s1  ,  l3*(s1*s2*s3 - c2*c3*s1),
    0.0                                        ,  - l3*(c2*s3 + c3*s2) - l2*s2          , -l3*(c2*s3 + c3*s2);

    exp_joint_vel = jocabain_exp_pos.ldlt().solve(exp_cart_vel); // 求得关节空间角速度

    return exp_joint_vel;
}

Vector3D Leg::calculateExpJointTorque()
{
    if(!exp_omega_is_update)     //如果之前没有计算过速度，那么计算一次关节期望速度
    {
        calculateExpJointOmega();
    }

    if(exp_force_is_update)
        return exp_joint_tor;

    double s1 = std::sin(exp_joint_pos[0]);
    double c1 = std::cos(exp_joint_pos[0]);
    double s2 = std::sin(exp_joint_pos[1]);
    double c2 = std::cos(exp_joint_pos[1]);
    double s3 = std::sin(exp_joint_pos[2]);
    double c3 = std::cos(exp_joint_pos[2]);

    double l0=param.l0;
    double d2 = param.d2; // 机械臂参数:长度，质心长度，质量
    double l2 = param.l2;
    double l3 = param.l3;
    double l1 = param.l1;

    Eigen::Matrix3d j_dq1, j_dq2, j_dq3;

    // ∂J/∂q1
    j_dq1 <<
            l3*(c2*s1*s3 + c3*s1*s2) + d2*c1 + l2*s1*s2 , - l3*(c1*c2*c3 - c1*s2*s3) - l2*c1*c2 , -l3*(c1*c2*c3 - c1*s2*s3),
            d2*s1 - l3*(c1*c2*s3 + c1*c3*s2) - l2*c1*s2 ,   l3*(s1*s2*s3 - c2*c3*s1) - l2*c2*s1 ,  l3*(s1*s2*s3 - c2*c3*s1),
            l3*s1*s3                                    ,   0.0                                 , -l3*c1*c3;

    // ∂J/∂q2
    j_dq2 <<
            - l3*(c1*c2*c3 - c1*s2*s3) - l2*c1*c2   ,   l3*(c2*s1*s3 + c3*s1*s2) + l2*s1*s2 ,  l3*(c2*s1*s3 + c3*s1*s2),
            l3*(s1*s2*s3 - c2*c3*s1) - l2*c2*s1     , - l3*(c1*c2*s3 + c1*c3*s2) - l2*c1*s2 , -l3*(c1*c2*s3 + c1*c3*s2),
            0.0                                     ,    - l2*c2 - l3*c2*c3                 , l3*s2*s3;
    // ∂J/∂q3
    j_dq3 <<
            -l3*(c1*c2*c3 - c1*s2*s3)   ,  l3*(c2*s1*s3 + c3*s1*s2) ,  l3*(c2*s1*s3 + c3*s1*s2),
            l3*(s1*s2*s3 - c2*c3*s1)    , -l3*(c1*c2*s3 + c1*c3*s2) , -l3*(c1*c2*s3 + c1*c3*s2),
            -l3*c1*c3                   , l3*s2*s3                  , -l3*(c2*c3 - s1*s3);

    Eigen::Matrix3d j_dq = exp_joint_vel[0] * j_dq1 + exp_joint_vel[1] * j_dq2 + exp_joint_vel[2] * j_dq3;
    Eigen::Vector3d exp_joint_acc = jocabain_exp_pos.ldlt().solve(exp_cart_acc - j_dq * exp_joint_vel);


    Eigen::Matrix3d R_1to0, R_2to1, R_3to2;
    R_1to0 <<
    c1, -s1, 0.0,
    s1, c1, 0.0,
    0.0, 0.0, 1.0;
    R_2to1 <<
    -s2,-c2,0.0,
    0.0,0.0,-1.0,
    c2,-s2,0.0;
    R_3to2 <<
    c3, -s3, 0.0,
    s3, c3, 0.0,
    0.0, 0.0, 1.0;

    // 关节旋转向量
    Vector3D joint1_acc = Vector3D(0.0, 0.0, exp_joint_acc[0]);
    Vector3D joint1_vel = Vector3D(0.0, 0.0, exp_joint_vel[0]);
    Vector3D joint2_acc = Vector3D(0.0, 0.0, exp_joint_acc[1]);
    Vector3D joint2_vel = Vector3D(0.0, 0.0, exp_joint_vel[1]);
    Vector3D joint3_acc = Vector3D(0.0, 0.0, exp_joint_acc[2]);
    Vector3D joint3_vel = Vector3D(0.0, 0.0, exp_joint_vel[2]);

    Vector3D ac1 = Vector3D(0.0, 0.0, 0.0); // 连杆1原点加速度为0
    Vector3D ag1 = Vector3D(-9.80, 0.0, 0.0); // 连杆1质心加速度为0

    Vector3D ac2 = R_2to1.transpose() * (ac1 + joint1_acc.cross(Vector3D(0.0,0.0,l1)) + joint1_vel.cross(joint1_vel.cross(Vector3D(0.0,0.0,l1)))); // 求连杆原点加速度
    Vector3D ag2 = ac2 + joint2_acc.cross(param.r2) + joint2_vel.cross(joint2_vel.cross(param.r2));                        // 求连杆质心线加速度

    Vector3D ac3 = R_3to2.transpose() * (ac2 + joint2_acc.cross(Vector3D(l2,0.0,d2)) + joint2_vel.cross(joint2_vel.cross(Vector3D(l2,0.0,d2))));
    Vector3D ag3 = ac3 + joint3_acc.cross(param.r3) + joint3_vel.cross(joint3_vel.cross(param.r3));

    // 牛顿-欧拉正向递推完成，开始进行动力学反推

    // 第3节
    Vector3D f3=exp_cart_for+param.m3*ag3;
    Vector3D n3=param.r3.cross(param.m3*ag3)+Vector3D(l3,0.0,0.0).cross(f3)+param.I3*joint3_acc+joint3_vel.cross(param.I3*joint3_vel);

    //第2节
    Vector3D f2=R_3to2*f3+param.m2*ag2;
    Vector3D n2=R_3to2*n3+param.r2.cross(param.m2*ag2)+Vector3D(l2,0.0,d2).cross(R_3to2*f3)+param.I2*joint2_acc+joint2_vel.cross(param.I2*joint2_vel);

    //第1节
    Vector3D f1=R_2to1*f2+param.m1*ag1;
    Vector3D n1=R_2to1*n2+param.r1.cross(param.m1*ag1)+Vector3D(0.0,0.0,l1).cross(R_2to1*f2)+param.I1*joint1_acc+joint1_vel.cross(param.I1*joint1_vel);

    Vector3D vz(0.0, 0.0, 1.0);

    exp_joint_tor << n1.transpose().dot(vz) , n2.transpose().dot(vz), n3.transpose().dot(vz);

    exp_torque_is_update=true;

    return exp_joint_tor;
}

Vector3D Leg::calculateCurFootPosition()
{
    Eigen::Vector4d result=param.T_GndToBase*cur_cart_pos.homogeneous();
    return Vector3D(result[0],result[1],result[2]);
}

Vector3D Leg::calculateCurFootVelocity()
{
    return param.R_GndToBase*(jocabain_cur_pos*cur_joint_vel);  //将坐标系转为支撑相中性点坐标系
}

Vector3D Leg::calculateCurFootForce()
{
    if(exp_torque_is_update)     //如果之前没计算过足端期望力矩，那么计算一次足端力矩，用于传感足端真实受力
        calculateExpJointTorque();
    cur_cart_for=jocabain_cur_pos.transpose().ldlt().solve(cur_joint_tor - exp_joint_tor);
    return param.R_GndToBase*cur_cart_for;  //将坐标系转为支撑相中性点坐标系
}
