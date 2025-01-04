//
// Created by lly on 2025/7/04.
//
#include <stdbool.h>
#include <stdlib.h>
#include "Dji.h"
#include "bsp_can.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "fdcan.h"
#include "Dji_Init.h"
extern float vofa_trans[32];
/***************************************************************
  *  @author        lly
  *  @brief         Detect whether the motor is stalled
  *  @param         Motor Struct
  *  @note
  *  @Sample usage: put it in Dji Callback
 **************************************************************/
void Dji_Stall_Monitoring(Dji_Control *Dji_Controler)
{
    if (Dji_Controler->Dji_Pid.Tar_Cur > 2000 && Dji_Controler->Dji_Cb.Cb_Torque > 1000 && Dji_Controler->Dji_Pid.Cur_Vel < 500) {
        Dji_Controler->flag_suspected_stall = true;
    }

    else {
        Dji_Controler->flag_suspected_stall = false;
        Dji_Controler->suspected_stall_time = 0;
    }

    if (Dji_Controler->flag_suspected_stall == true) {
        Dji_Controler->suspected_stall_time += 1;
        if (Dji_Controler->suspected_stall_time > stall_time) {
//            Dji_Controler->flag_is_stall = true;
        }
    }

    if (Dji_Controler->flag_is_stall == true) ///
    {
        Dji_Controler->warning_time_cnt += 1;
        if (Dji_Controler->warning_time_cnt >= 100) {
            Dji_Controler->warning_time_cnt = 0;
            HAL_GPIO_TogglePin(warning_led_Port, warning_led_Pin);
        }
    }
}

/***************************************************************
  *  @author        lly
  *  @brief         change motor pid param
  *  @param         Motor Struct, pid param
  *  @note
  *  @Sample usage:
 **************************************************************/
void Dji_Setparam(Dji_Control *Dji_Controler, uint16_t Kp_vel, uint16_t Ki_vel, uint16_t Kd_vel, uint16_t Kp_pos, uint16_t Ki_pos, uint16_t Kd_pos)
{
    Dji_Controler->Dji_Pid.vel_Kp = Kp_vel;
    Dji_Controler->Dji_Pid.vel_Ki = Ki_vel;
    Dji_Controler->Dji_Pid.vel_Kd = Kd_vel;
    Dji_Controler->Dji_Pid.pos_Kp = Kp_pos;
    Dji_Controler->Dji_Pid.pos_Ki = Ki_pos;
    Dji_Controler->Dji_Pid.pos_Kd = Kd_pos;
}

/***************************************************************
  *  @author        lly
  *  @brief         Get motor current total position
  *  @param         Motor Struct
  *  @note
  *  @Sample usage: put it in Dji Callback
 **************************************************************/
void Dji_GetPosition(Dji_Control *Dji_Controler)
{
    static int32_t measure = 0, dis1 = 0, dis2 = 0;
    measure = Dji_Controler->Dji_Cb.Cb_EncPosi;

    if (measure < Dji_Controler->Dji_Pid.last_measure) {
        dis1 = measure + 8191 - Dji_Controler->Dji_Pid.last_measure; /// positive
        dis2 = measure - Dji_Controler->Dji_Pid.last_measure;        /// negative
    } else {
        dis1 = measure - 8191 - Dji_Controler->Dji_Pid.last_measure; /// positive
        dis2 = measure - Dji_Controler->Dji_Pid.last_measure;        /// negative
    }

    Dji_Controler->Dji_Pid.last_measure = measure;

    if (abs(dis1) < abs(dis2)) {
        Dji_Controler->Dji_Pid.cnt_angle += dis1;
    } else {
        Dji_Controler->Dji_Pid.cnt_angle += dis2;
    }
    Dji_Controler->Dji_Pid.Cur_Vel = Dji_Controler->Dji_Cb.Cb_Vel;
    switch (Dji_Controler->Motor_Mode) {
        case Mode_M3508: {
            Dji_Controler->Dji_Pid.Cur_Pos = (float)(Dji_Controler->Dji_Pid.cnt_angle) * 0.000122f / 19.f;
        } break;
        case Mode_M2006: {
            Dji_Controler->Dji_Pid.Cur_Pos = (float)(Dji_Controler->Dji_Pid.cnt_angle) * 0.000122f / 36.f;
        } break;
    }
}

/***************************************************************
  *  @author        lly
  *  @brief         Velcity pid
  *  @param         Motor Struct
  *  @note
  *  @Sample usage: put it in Dji Callback
 **************************************************************/
void Pid_Velocity(Dji_Control *Dji_Controler)
{
    static int32_t last_error = 0, Out_Cur = 0;
    int32_t Vel_err = Dji_Controler->Dji_Pid.Tar_Vel - Dji_Controler->Dji_Pid.Cur_Vel;

    Dji_Controler->Dji_Pid.vel_proportion = Dji_Controler->Dji_Pid.vel_Kp * Vel_err;

    Dji_Controler->Dji_Pid.vel_proportion = Dji_Controler->Dji_Pid.vel_proportion > 9500 ? 9500 : Dji_Controler->Dji_Pid.vel_proportion;
    Dji_Controler->Dji_Pid.vel_proportion = Dji_Controler->Dji_Pid.vel_proportion < -9500 ? -9500 : Dji_Controler->Dji_Pid.vel_proportion;

    Dji_Controler->Dji_Pid.vel_differential = Dji_Controler->Dji_Pid.vel_Kd * (Vel_err - last_error);
    last_error                              = Vel_err;
    Out_Cur                                 = Dji_Controler->Dji_Pid.vel_proportion + Dji_Controler->Dji_Pid.vel_intergration + Dji_Controler->Dji_Pid.vel_differential;

    Out_Cur = Out_Cur > Vel_total_limit ? Vel_total_limit : Out_Cur;
    Out_Cur = Out_Cur < -Vel_total_limit ? -Vel_total_limit : Out_Cur;

    bool limit = false;
    if (Out_Cur >= Vel_total_limit || Out_Cur <= -Vel_total_limit) limit = true;

    if (limit)
        Dji_Controler->Dji_Pid.vel_intergration *= 0.99f;
    else
        Dji_Controler->Dji_Pid.vel_intergration += current_mea_period * Vel_err * Dji_Controler->Dji_Pid.vel_Ki;
    Dji_Controler->Dji_Pid.Tar_Cur = Out_Cur;
}


/***************************************************************
  *  @author        lly
  *  @brief         Position pid
  *  @param         Motor Struct
  *  @note
  *  @Sample usage: put it in Dji Callback
 **************************************************************/
void Pid_Position(Dji_Control *Dji_Controler)
{
    static float last_error = 0, Out_Vel = 0;
    float Pos_err = Dji_Controler->Dji_Pid.Tar_Pos - Dji_Controler->Dji_Pid.Cur_Pos;

    Dji_Controler->Dji_Pid.pos_proportion   = Dji_Controler->Dji_Pid.pos_Kp * Pos_err;
    Dji_Controler->Dji_Pid.pos_proportion   = Dji_Controler->Dji_Pid.pos_proportion > 30000 ? 30000 : Dji_Controler->Dji_Pid.pos_proportion;
    Dji_Controler->Dji_Pid.pos_proportion   = Dji_Controler->Dji_Pid.pos_proportion < -30000 ? -30000 : Dji_Controler->Dji_Pid.pos_proportion;
    Dji_Controler->Dji_Pid.pos_differential = Dji_Controler->Dji_Pid.pos_Kd * (last_error - Pos_err);
    last_error                              = Pos_err;
    Out_Vel                                 = Dji_Controler->Dji_Pid.pos_proportion + Dji_Controler->Dji_Pid.pos_intergration + Dji_Controler->Dji_Pid.pos_differential;

    Out_Vel = Out_Vel > Pos_total_limit ? Pos_total_limit : Out_Vel;
    Out_Vel = Out_Vel < -Pos_total_limit ? -Pos_total_limit : Out_Vel;

    bool limit = false;
    if (Out_Vel >= Pos_total_limit || Out_Vel <= -Pos_total_limit) limit = true;

    if (limit)
        Dji_Controler->Dji_Pid.pos_intergration *= 0.99f;
    else
        Dji_Controler->Dji_Pid.pos_intergration += current_mea_period * Pos_err * Dji_Controler->Dji_Pid.pos_Ki;
    Dji_Controler->Dji_Pid.Tar_Vel = Out_Vel;

}


/***************************************************************
  *  @author        lly
  *  @brief         Motor all callback
  *  @param         Motor Struct, can Rxhander ,can rx buffer ,Tx Id
  *  @note
  *  @Sample usage: put it in Dji Callback
 **************************************************************/

void Dji_Callback(FDCAN_HandleTypeDef *hfdcan,Dji_Control *Dji_Controler, FDCAN_RxHeaderTypeDef RxHeaderr, const uint8_t *Rxbufferr, uint16_t Tx_ID)
{

    Dji_Controler->Dji_Cb.Cb_EncPosi = (int16_t)((Rxbufferr[0]) << 8 | (Rxbufferr[1]));
    Dji_Controler->Dji_Cb.Cb_Vel     = (int16_t)((Rxbufferr[2]) << 8 | (Rxbufferr[3]));
    Dji_Controler->Dji_Cb.Cb_Torque  = (int16_t)((Rxbufferr[4]) << 8 | (Rxbufferr[5]));
    Dji_Controler->Dji_Cb.Cb_Tem     = Rxbufferr[7];
    Dji_Controler->Dji_Pid.Cur_Vel   =Dji_Controler->Dji_Cb.Cb_Vel;
    Dji_GetPosition(Dji_Controler);

//    Dji_Stall_Monitoring(Dji_Controler);
//    if (Dji_Controler->flag_is_stall == true) {
//        Dji_Controler->Dji_Pid.Tar_Cur = 0;
//    }

    switch (Dji_Controler->Control_Mode) {
        case Mode_Pos: {
            Pid_Position(Dji_Controler);
        }

        case Mode_Vel: {
            Pid_Velocity(Dji_Controler);
        } break;

        case Mode_Null: {
        }
    }

    if(hfdcan->Instance == FDCAN1) {
        static uint8_t Dji_TxDate_1[8];
        switch (Tx_ID) {
            case 0x200: {
                Dji_TxDate_1[((RxHeaderr.Identifier % 16) - 1) * 2] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur >> 8);
                Dji_TxDate_1[((RxHeaderr.Identifier % 16) - 1) * 2 + 1] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur);
            }
                break;
            case 0x1ff: {
                Dji_TxDate_1[((RxHeaderr.Identifier % 16-4) - 1) * 2] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur >> 8);
                Dji_TxDate_1[((RxHeaderr.Identifier % 16-4) - 1) * 2 + 1] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur);
            }
                break;
        }
        fdcan_send_msg(&hfdcan1, Tx_ID, 8, Dji_TxDate_1);
    }

    if(hfdcan->Instance == FDCAN2) {
        static uint8_t Dji_TxDate_2[8];
        switch (Tx_ID) {
            case 0x200: {
                Dji_TxDate_2[((RxHeaderr.Identifier % 16) - 1) * 2] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur >> 8);
                Dji_TxDate_2[((RxHeaderr.Identifier % 16) - 1) * 2 + 1] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur);
            }
                break;
            case 0x1ff: {
                Dji_TxDate_2[((RxHeaderr.Identifier % 16-4) % 4 - 1) * 2] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur >> 8);
                Dji_TxDate_2[((RxHeaderr.Identifier % 16-4) % 4 - 1) * 2 + 1] = (uint8_t) (Dji_Controler->Dji_Pid.Tar_Cur);
            }
                break;
        }
        fdcan_send_msg(&hfdcan2, Tx_ID, 8, Dji_TxDate_2);
    }


}


/***************************************************************
  *  @author        lly
  *  @brief
  *  @param
  *  @note
  *  @Sample usage: put it in can callback
 **************************************************************/
void Dji_Can_fifo0(FDCAN_HandleTypeDef *hfdcan,FDCAN_RxHeaderTypeDef RxHeaderrr,uint8_t RX_DATA_Dji[])
{
#if defined(hcan11)
    if (hfdcan->Instance == FDCAN1){
//        usb_printf("can1:%X\r\n",RxHeaderrr.Identifier);
        switch (RxHeaderrr.Identifier) {
#if defined(Dji_0x201_can1)
            case 0x201: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x201, RxHeaderrr, RX_DATA, 0x200);
            } break;
#endif

#if defined(Dji_0x202_can1)
            case 0x202: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x202, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x203_can1)
            case 0x203: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x203, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x204_can1)
            case 0x204: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x204, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x205_can1)
            case 0x205: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x205, RxHeaderrr, RX_DATA, 0x1ff);
            } break;
#endif

#if defined(Dji_0x206_can1)
            case 0x206: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x206, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif

#if defined(Dji_0x207_can1)
            case 0x207: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x207, RxHeaderrr, RX_DATA, 0x1ff);
            } break;
#endif

#if defined(Dji_0x208_can1)
            case 0x208: {
                Dji_Callback(&hfdcan1,&Dji_Control_can1_0x208, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif
        }
    }
#endif

#if defined(hcan22)
    if (hfdcan->Instance == FDCAN2) {
        switch (RxHeaderrr.Identifier) {
#if defined(Dji_0x201_can2)
            case 0x201: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x201, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;0.0
#endif
#if defined(Dji_0x202_can2)
            case 0x202: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x202, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x203_can2)
            case 0x203: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x203, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x204_can2)
            case 0x204: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x204, RxHeaderrr, RX_DATA_Dji, 0x200);
            } break;
#endif

#if defined(Dji_0x205_can2)
            case 0x205: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x205, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif

#if defined(Dji_0x206_can2)
            case 0x206: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x206, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif

#if defined(Dji_0x207_can2)
            case 0x207: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x207, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif

#if defined(Dji_0x208_can2)
            case 0x208: {
                Dji_Callback(&hfdcan2,&Dji_Control_can2_0x208, RxHeaderrr, RX_DATA_Dji, 0x1ff);
            } break;
#endif
        }
    }
#endif
}
