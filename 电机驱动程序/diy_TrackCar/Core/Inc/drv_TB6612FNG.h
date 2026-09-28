/*
 * drv_TB6612FNG.h
 *
 *  Created on: 2026年8月28日
 *      Author: 27059
 */

#ifndef INC_DRV_TB6612FNG_H_
#define INC_DRV_TB6612FNG_H_

#include "main.h"

// 速度范围 0 ~ 999
#define TB6612_SPEED_MAX     999
#define TB6612_SPEED_MIN     0

// 电机方向定义
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
}Motor_DirTypeDef;

void TB6612_Init(void);
// 左右电机独立控制 l_speed/r_speed:0‑999
void TB6612_SetMotor(int16_t l_speed, int16_t r_speed);

// 小车动作接口
void TB6612_Car_Forward(uint16_t speed);    //前进
void TB6612_Car_Backward(uint16_t speed);   //后退
void TB6612_Car_TurnLeft(uint16_t speed);   //原地左转
void TB6612_Car_TurnRight(uint16_t speed);  //原地右转
void TB6612_Car_SpinLeft(uint16_t speed);   //原地掉头左转
void TB6612_Car_SpinRight(uint16_t speed);  //原地掉头右转
void TB6612_Car_Stop(void);                 //整车停止
void TB6612_Car_Left_Right(int16_t lspeed,int16_t rspeed);	//左右同时给定

#endif /* INC_DRV_TB6612FNG_H_ */
