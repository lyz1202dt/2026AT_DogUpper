#include "step.h"
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

static inline void set_quintic(CubicLineParam_t &seg,double p0, double v0, double a0,double pT, double vT, double aT,float dt)
{
    double T = dt;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    seg.a = p0;
    seg.b = v0;
    seg.c = 0.5 * a0;

    // 五次多项式：p(t) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5
    // 边界条件：p(0)=p0, v(0)=v0, a(0)=a0, p(T)=pT, v(T)=vT, a(T)=aT
    // 解方程得到：
    seg.d = ( 10*(pT - p0) - (6*v0 + 4*vT)*T - (1.5*a0 - 0.5*aT)*T2 ) / T3;
    seg.e = ( -15*(pT - p0) + (8*v0 + 7*vT)*T + (1.5*a0 - aT)*T2 ) / T4;
    seg.f = ( 6*(pT - p0) - (3*v0 + 3*vT)*T - (0.5*a0 - 0.5*aT)*T2 ) / T5;
}

static inline float get_quintic_value(const CubicLineParam_t &line,const float time)
{
    return line.a + line.b * time + line.c * time * time +line.d * time * time * time + line.e * time*time*time*time + line.f * time*time*time*time*time;
}

static inline float get_quintic_dt(const CubicLineParam_t &line,const float time)
{
    return line.b + 2.0f * line.c * time + 3.0f * line.d * time * time +4.0f * line.e * time*time*time + 5.0f * line.f * time*time*time*time;
}

static inline float get_quintic_dtdt(const CubicLineParam_t &line,const float time)
{
    return 2.0f * line.c + 6.0f * line.d * time +12.0f * line.e * time * time + 20.0f * line.f * time*time*time;
}

static inline std::tuple<Vector3D, Vector3D, Vector3D> CalculateTarget_Air(Trajectory_t *line,float time)
{
    Vector3D pos;
    Vector3D vel;
    Vector3D acc;

    float local_time=time-line->time*0.5f;                     //重映射时间从0.5T~T到0~0.5T

    // X 方向五次多项式
    pos[0] = get_quintic_value(line->lx,local_time);
    vel[0] = get_quintic_dt(line->lx,local_time);
    acc[0] = get_quintic_dtdt(line->lx,local_time);

    // Y 方向五次多项式
    pos[1] = get_quintic_value(line->ly,local_time);
    vel[1] = get_quintic_dt(line->ly,local_time);
    acc[1] = get_quintic_dtdt(line->ly,local_time);

    // Z 方向分两段（前半抬腿，后半落腿）
    if (local_time<line->time*0.25f)
    {
        pos[2] = get_quintic_value(line->l1_z,local_time);
        vel[2] = get_quintic_dt(line->l1_z,local_time);
        acc[2] = get_quintic_dtdt(line->l1_z,local_time);
        //RCLCPP_INFO(rclcpp::get_logger("step_line"),"前半程,时间%f",local_time);
    }
    else
    {
        pos[2] = get_quintic_value(line->l2_z,local_time-line->time*0.25f);
        vel[2] = get_quintic_dt(line->l2_z,local_time-line->time*0.25f);
        acc[2] = get_quintic_dtdt(line->l2_z,local_time-line->time*0.25f);
        //RCLCPP_INFO(rclcpp::get_logger("step_line"),"后半程,时间%f",local_time);
    }

    // 注意：vel 和 acc 需要按实际时间缩放
    return std::make_tuple(pos, vel, acc);
}

static inline std::tuple<Vector3D, Vector3D, Vector3D> CalculateTarget_Gnd(Trajectory_t *line, float time)
{
    Vector3D pos;
    Vector3D vel;

    pos[0] = line->gx.b + line->gx.k * time;
    vel[0] = line->gx.k / time;

    pos[1] = line->gy.b + line->gy.k * time;
    vel[1] = line->gy.k / time;

    pos[2]=0.0;
    vel[2]=0.0;

    return std::make_tuple(pos,vel,Vector3D(0.0,0.0,0.0));
}

std::tuple<Vector3D, Vector3D, Vector3D> GetLegTarget(float time, Trajectory_t &line)
{
    float step=time/line.time;

    if (step > 1.0f)
        step = 1.0f;
    else if (step < 0.0f)
        step = 0.0f;

    if (step <= 0.5f) // 支撑相
        return CalculateTarget_Gnd(&line,time);
    else //摆动相
        return CalculateTarget_Air(&line,time);
}


bool UpdateGndStepLine(const Vector3D &cur_pos, const Vector2D &exp_vel, Trajectory_t *line,float time)
{
    double target_x = exp_vel[0]*time*0.25; //理想情况下，足端轨迹中心应该过足端坐标系的中点
    double target_y = exp_vel[1]*time*0.25;

    line->time=time;    //记录一个步态相位的时间

    line->gx.k=(-target_x-cur_pos[0])/(time*0.5);
    line->gx.b=cur_pos[0];

    line->gy.k=(-target_y-cur_pos[1])/(time*0.5);    //对于支撑相，之后的运动应为从当前点到到步态结束点的直线
    line->gy.b=cur_pos[1];

    return true;
}

bool UpdateAirStepLine(const Vector3D &cur_pos, const Vector3D &cur_vel, const Vector2D &exp_vel, Trajectory_t *line,float time,float step_height)
{

    double target_x = exp_vel[0]*time*0.25; //理想情况下，足端落点位置为期望的移动速度*整个步态时间*0.5*0.5
    double target_y = exp_vel[1]*time*0.25;
    

    set_quintic(line->lx,
                cur_pos[0], cur_vel[0], 0.0,   // 起点
                target_x,  -exp_vel[0], 0.0, time*0.5f);  // 终点
    // y方向轨迹
    set_quintic(line->ly,
                cur_pos[1], cur_vel[1], 0.0,
                target_y,  -exp_vel[1], 0.0, time*0.5f);
    // z方向分为两段：抬腿 -> 落腿
    // 第一段：从当前z抬到最高点
    set_quintic(line->l1_z,
                cur_pos[2], cur_vel[2], 0.0,
                step_height, 0.0, 0.0, time*0.25f);

    // 第二段：从最高点落到地面
    set_quintic(line->l2_z,
                step_height, 0.0, 0.0,
                0.0, 0.0, 0.0, time*0.25f);

    return true;
}
