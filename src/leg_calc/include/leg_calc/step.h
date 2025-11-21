#ifndef __STEP_H__
#define __STEP_H__

#include "leg.h"
#include <tuple>

typedef Eigen::Vector2d Vector2D;

typedef struct
{
    double a;
    double b;
    double c;
    double d;
    double e;
    double f;
}CubicLineParam_t;

typedef struct
{
    double k;
    double b;
}StraightLineParam_t;


typedef struct
{
    CubicLineParam_t lx;
    CubicLineParam_t ly;
    CubicLineParam_t l1_z;
    CubicLineParam_t l2_z;
    StraightLineParam_t gx;
    StraightLineParam_t gy;
    float time;
}Trajectory_t;

typedef struct
{
    double exp_vx;
    double exp_vy;
    double H;    //摆动轨迹半径
    double Lx;    //x方向步长
    double Ly;    //x方向步长
    float T;
}CycloidStep_t;

//规划步态，在step==0.5或==1时调用更新步态，一定返回true
bool UpdateGndStepLine(const Vector3D &cur_pos, const Vector2D &exp_vel, Trajectory_t *line,float time);
bool UpdateAirStepLine(const Vector3D &cur_pos, const Vector3D &cur_vel, const Vector2D &exp_vel, Trajectory_t *line,float time,float step_height);
//在每次迭代中，返回电机的期望位置，期望速度，期望力矩
std::tuple<Vector3D, Vector3D, Vector3D> GetLegTarget(float time, Trajectory_t &line);

bool UpdateCycloidStep(const Vector2D &exp_vel, CycloidStep_t *line,float time,float step_height);
std::tuple<Vector3D,Vector3D,Vector3D> GetCycloidStep(float time, CycloidStep_t &line);


#endif