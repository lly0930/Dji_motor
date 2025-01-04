//
// Created by 29787 on 2024/5/28.
//
#include "Dji.h"
#ifndef MOTOR_CTRL_DJI_INIT_H
#define MOTOR_CTRL_DJI_INIT_H

#define hcan11
#define hcan22

//#define Dji_0x201_can1
#define Dji_0x202_can1
#define Dji_0x203_can1
#define Dji_0x204_can1
//#define Dji_0x205_can1
//#define Dji_0x206_can1
//#define Dji_0x207_can1
//#define Dji_0x208_can1

//#define Dji_0x201_can2
#define Dji_0x202_can2
#define Dji_0x203_can2
#define Dji_0x204_can2
//#define Dji_0x205_can2
//#define Dji_0x206_can2
//#define Dji_0x207_can2
//#define Dji_0x208_can2


/////////////////////////////////////////can1//////////////////////////////////
#if defined(Dji_0x201_can1)
extern Dji_Control Dji_Control_can1_0x201;
#endif

#if defined(Dji_0x202_can1)
extern Dji_Control Dji_Control_can1_0x202;
#endif

#if defined(Dji_0x203_can1)
extern Dji_Control Dji_Control_can1_0x203;
#endif

#if defined(Dji_0x204_can1)
extern Dji_Control Dji_Control_can1_0x204;
#endif

#if defined(Dji_0x205_can1)
extern Dji_Control Dji_Control_can1_0x205;
#endif

#if defined(Dji_0x206_can1)
extern Dji_Control Dji_Control_can1_0x206;
#endif

#if defined(Dji_0x207_can1)
extern Dji_Control Dji_Control_can1_0x207;
#endif

#if defined(Dji_0x208_can1)
extern Dji_Control Dji_Control_can1_0x208;
#endif

//////////////////////////////////////////can2//////////////////////////
#if defined(Dji_0x201_can2)
extern Dji_Control Dji_Control_can2_0x201;
#endif

#if defined(Dji_0x202_can2)
extern Dji_Control Dji_Control_can2_0x202;
#endif

#if defined(Dji_0x203_can2)
extern Dji_Control Dji_Control_can2_0x203;
#endif

#if defined(Dji_0x204_can2)
extern Dji_Control Dji_Control_can2_0x204;
#endif

#if defined(Dji_0x205_can2)
extern Dji_Control Dji_Control_can2_0x205;
#endif

#if defined(Dji_0x206_can2)
extern Dji_Control Dji_Control_can2_0x206;
#endif

#if defined(Dji_0x207_can2)
extern Dji_Control Dji_Control_can2_0x207;
#endif

#if defined(Dji_0x208_can2)
extern Dji_Control Dji_Control_can2_0x208;
#endif

#endif // MOTOR_CTRL_DJI_INIT_H
