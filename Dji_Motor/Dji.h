//
// Created by 29787 on 2024/5/27.
//
#include "stdbool.h"
#ifndef MOTOR_CTRL_DJI_H
#define MOTOR_CTRL_DJI_H
#include "fdcan.h"
#include "stdint.h"
///--------------macro definition-----------------------------------
#define Vel_total_limit    9500
#define Pos_total_limit    30000
#define Cur_total_limit    10000
#define current_mea_period 0.001
#define stall_time         2000

///-------------------led definition--------------------------------
#define running_led_Port  LED1_GPIO_Port
#define running_led_Pin   LED1_Pin
#define warning_led_Port  LED2_GPIO_Port
#define warning_led_Pin   LED2_Pin
///------------------struct-----------------------------------------
typedef enum {
    Mode_Vel=0,
    Mode_Pos,
    Mode_Cur,
    Mode_Null,
}Motor_Control_Mode;

typedef enum{
    Mode_M2006,
    Mode_M3508,
}Motor_Mode;

typedef struct {
    int16_t Tar_Vel;
    float   Tar_Pos;
    int16_t Tar_Cur;
    int16_t Cur_Vel;
    float   Cur_Pos;

    float pos_Kp;
    float pos_Ki;
    float pos_Kd;

    float vel_Kp;
    float vel_Ki;
    float vel_Kd;

    float vel_proportion;
    float vel_intergration;
    float vel_differential;

    float pos_proportion;
    float pos_intergration;
    float pos_differential;

    float last_measure;
    float cnt_angle;
}Dji_Pid_;


typedef struct {
    int16_t  Cb_EncPosi;
    int16_t  Cb_Vel;
    int16_t  Cb_Torque;
    uint8_t  Cb_Tem;
}Dji_Cb_;

typedef struct {
    uint8_t   Motor_Mode;
    uint8_t   Control_Mode;
    bool      flag_is_stall;
    bool      flag_suspected_stall;
    uint32_t  warning_time_cnt;
    uint32_t  suspected_stall_time;
    Dji_Cb_   Dji_Cb;
    Dji_Pid_  Dji_Pid;
}Dji_Control;


///-----------------function declaration-------------------------------
void Dji_Can_fifo0(FDCAN_HandleTypeDef *hfdcan,FDCAN_RxHeaderTypeDef RxHeaderrr,uint8_t RX_DATA[]);
void Pid_Velocity   (Dji_Control* Dji_Controler);
void Pid_Position   (Dji_Control* Dji_Controler);
void Dji_GetPosition(Dji_Control* Dji_Controler);
void Dji_Stall_Monitoring(Dji_Control *Dji_Controler);
void Dji_Setparam(Dji_Control *Dji_Controler,uint16_t Kp_vel,uint16_t Ki_vel,uint16_t Kd_vel,uint16_t Kp_pos,uint16_t Ki_pos,uint16_t Kd_pos);
void Dji_Callback   (FDCAN_HandleTypeDef *hfdcan,Dji_Control* Dji_Control_0x201,FDCAN_RxHeaderTypeDef RxHeaderr,const uint8_t* Rxbuffer, uint16_t Tx_ID);
#endif // MOTOR_CTRL_DJI_H
