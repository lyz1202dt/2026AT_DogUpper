#include "vmc.hpp"
#include <cassert>
#include <chrono>
#include <algorithm>

VMC::VMC(double kp,double kd,double mass,double max_acc,double max_vel,double max_pos,std::chrono::high_resolution_clock::duration dt,
        double start_pos,double start_vel)
{
    this->kp=kp;
    this->kd=kd;
    this->mass=mass;
    this->max_acc=max_acc;
    this->max_vel=max_vel;
    this->max_pos=max_pos;
    this->virtual_pos=start_pos;
    this->virtual_vel=start_vel;
    this->dt= std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
}

std::tuple<double,double,double> VMC::targetUpdate(double exp_pos,double cur_pos,double exp_vel,double cur_vel,double cur_force)
{
    double virtual_force =  kp * (exp_pos - cur_pos)  + kd * (exp_vel - cur_vel) + cur_force;

    double acc = virtual_force / mass;

    // 3. 加速度限幅
    acc = std::clamp(acc, -max_acc, max_acc);

    // 4. 积分得到虚拟位置和速度
    virtual_pos=virtual_pos + virtual_vel * dt + 0.5 * acc * dt * dt;
    virtual_vel=virtual_vel+ acc * dt;

    // 5. 位置和速度限幅
    virtual_pos = std::clamp(virtual_pos, -max_pos, max_pos);
    virtual_vel = std::clamp(virtual_vel, -max_vel, max_vel);

    // 6. 返回虚拟位置、虚拟速度、期望力
    return std::make_tuple(virtual_pos, virtual_vel, acc);
}

void VMC::reset(double virtual_pos,double virtual_vel)
{
    this->virtual_pos=virtual_pos;
    this->virtual_vel=virtual_vel;
}