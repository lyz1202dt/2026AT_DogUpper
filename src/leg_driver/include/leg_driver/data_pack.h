#ifndef __DATAPACK_H__
#define __DATAPACK_H__

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorState_t;

typedef struct{
    MotorState_t joint[3];
}LegState_t;

typedef struct{
    int id;
    LegState_t leg;
}LegPack_t;

#pragma pack()

#endif