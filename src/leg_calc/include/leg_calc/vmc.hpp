#ifndef __VMC_H__
#define __VMC_H__

#include <chrono>
#include <cmath>
#include <tuple>

class VMC{
public:
    //虚拟质量-弹簧-阻尼系统参数
    VMC(double kp,double kd,double mass,double max_acc,double max_vel,double max_pos,std::chrono::high_resolution_clock::duration dt,
        double start_pos=0.0,double start_vel=0.0);
    //进行VMC计算，并返回新的期望位置/速度/力
    std::tuple<double,double,double> targetUpdate(double exp_pos,double cur_pos,double exp_vel,double cur_vel,double cur_force);
    double kp;
    double kd;
    double mass;
private:
    double virtual_pos;
    double virtual_vel;

    double max_pos;
    double max_vel;
    double max_acc;
    double dt;
};

#endif