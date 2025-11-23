#include "vmc.hpp"
#include <cassert>
#include <chrono>

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

std::tuple<double,double,double> VMC::VMC_handle(double pos,double vel,double force)
{
    double e  = virtual_pos - pos;
    double ed = virtual_vel - vel;

    double f = -kp*e - kd*ed + force;

    
    double acc=f/mass;

    if(acc>max_acc)
        acc=max_acc;
    else if(acc<-max_acc)
        acc=-max_acc;

    virtual_pos=virtual_pos+virtual_vel*dt+0.5*acc*dt*dt;
    if(virtual_pos>max_pos)
        virtual_pos=max_pos;
    else if(virtual_pos<-max_pos)
        virtual_pos=-max_pos;

    virtual_vel=virtual_vel+acc*dt;
    if(virtual_vel>max_vel)
        virtual_vel=max_vel;
    else if(virtual_vel<-max_vel)
        virtual_vel=-max_vel;

    return std::make_tuple(virtual_pos,virtual_vel,f);
}
