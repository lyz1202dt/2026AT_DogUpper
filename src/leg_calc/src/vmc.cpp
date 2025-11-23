#include "vmc.hpp"
#include <cassert>
#include <chrono>
#include <algorithm>

VMC::VMC(double kp,double kd,double mass,double max_acc,double max_vel,double max_pos,std::chrono::high_resolution_clock::duration dt,
        double cur_pos,double cur_vel)
{
    this->kp=kp;
    this->kd=kd;
    this->mass=mass;
    this->max_acc=max_acc;
    this->max_vel=max_vel;
    this->max_pos=max_pos;
    this->virtual_pos=cur_pos;
    this->virtual_vel=cur_vel;
    this->dt= std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
}

std::tuple<double,double,double> VMC::VMC_handle(double exp_pos,double cur_pos,double exp_vel,double cur_vel,double force)
{
    double virtual_force = + kp * (exp_pos - cur_pos)  + kd * (exp_vel - cur_vel) + force;

    double acc = virtual_force / mass;

    // 3. 加速度限幅
    acc = std::clamp(acc, -max_acc, max_acc);

    // 4. 积分得到虚拟位置和速度
    virtual_pos += virtual_vel * dt + 0.5 * acc * dt * dt;
    virtual_vel += acc * dt;

    // 5. 位置和速度限幅
    virtual_pos = std::clamp(virtual_pos, -max_pos, max_pos);
    virtual_vel = std::clamp(virtual_vel, -max_vel, max_vel);

    // 6. 返回虚拟位置、虚拟速度、限制后的力
    return std::make_tuple(virtual_pos, virtual_vel, acc * mass);
}
