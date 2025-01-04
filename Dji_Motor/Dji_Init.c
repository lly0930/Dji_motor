//
// Created by 29787 on 2024/5/28.
//
#include "Dji.h"
#include "Dji_Init.h"

/////////////////////////////////////////can1//////////////////////////////////
#if defined(Dji_0x201_can1)
Dji_Control Dji_Control_can1_0x201={
    .Control_Mode=Mode_Null,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=0,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,};
#endif

#if defined(Dji_0x202_can1)
Dji_Control Dji_Control_can1_0x202={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x203_can1)
Dji_Control Dji_Control_can1_0x203={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=8000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x204_can1)
Dji_Control Dji_Control_can1_0x204={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=7000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x205_can1)
Dji_Control Dji_Control_can1_0x205={
    .Control_Mode=Mode_Pos,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=10,   ///10
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000, ///8000
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x206_can1)
Dji_Control Dji_Control_can1_0x206={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x207_can1)
Dji_Control Dji_Control_can1_0x207={
    .Control_Mode=Mode_Null,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=0,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x208_can1)
Dji_Control Dji_Control_can1_0x208={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

//////////////////////////////////////////can2//////////////////////////
#if defined(Dji_0x201_can2)
Dji_Control Dji_Control_can2_0x201={
    .Control_Mode=Mode_Null,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=0,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x202_can2)
Dji_Control Dji_Control_can2_0x202={
    .Control_Mode=Mode_Pos,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=8000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x203_can2)
Dji_Control Dji_Control_can2_0x203={
    .Control_Mode=Mode_Pos,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x204_can2)
Dji_Control Dji_Control_can2_0x204={
    .Control_Mode=Mode_Pos,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=1000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x205_can2)
Dji_Control Dji_Control_can2_0x205={
    .Control_Mode=Mode_Null,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=0,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x206_can2)
Dji_Control Dji_Control_can2_0x206={
    .Control_Mode=Mode_Pos,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=5000,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x207_can2)
Dji_Control Dji_Control_can2_0x207={
    .Control_Mode=Mode_Null,
    .Motor_Mode=Mode_M3508,

    .Dji_Pid.vel_Kp=0,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif

#if defined(Dji_0x208_can2)
Dji_Control Dji_Control_can2_0x208={
    .Control_Mode=Mode_Vel,
    .Motor_Mode=Mode_M2006,

    .Dji_Pid.vel_Kp=10,
    .Dji_Pid.vel_Ki=0,
    .Dji_Pid.vel_Kd=0,

    .Dji_Pid.pos_Kp=0,
    .Dji_Pid.pos_Ki=0,
    .Dji_Pid.pos_Kd=0,
};
#endif