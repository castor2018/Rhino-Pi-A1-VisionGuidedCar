/*
 * drv_TB6612FNG.c
 *
 *  Created on: 2026年8月28日
 *      Author: 27059
 */


#include "drv_TB6612FNG.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

/**
 * @brief TB6612初始化，开启PWM，退出待机
 */
void TB6612_Init(void)
{
    //开启TIM1 CH1 CH2 PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    //STBY拉高，TB6612退出待机
    HAL_GPIO_WritePin(WHELL_STBY_GPIO_Port, WHELL_STBY_Pin, GPIO_PIN_SET);

    TB6612_Car_Stop();
}

/**
 * @brief 设置左右电机，正负代表方向，绝对值代表速度0‑999
 * @param l_speed 左电机： -999 ~ +999；正数前进，负数后退，0停止
 * @param r_speed 右电机： -999 ~ +999
 */
void TB6612_SetMotor(int16_t l_speed, int16_t r_speed)
{
    //限幅
    if(l_speed > TB6612_SPEED_MAX) l_speed = TB6612_SPEED_MAX;
    if(l_speed < -TB6612_SPEED_MAX) l_speed = -TB6612_SPEED_MAX;
    if(r_speed > TB6612_SPEED_MAX) r_speed = TB6612_SPEED_MAX;
    if(r_speed < -TB6612_SPEED_MAX) r_speed = -TB6612_SPEED_MAX;

    // ==========左电机 PB0 PB1==========
    if(l_speed > 0)
    {
        //左电机前进 IN0=0 IN1=1
        HAL_GPIO_WritePin(LEFT_WHELL_IN0_GPIO_Port, LEFT_WHELL_IN0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_WHELL_IN1_GPIO_Port, LEFT_WHELL_IN1_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, l_speed);
    }
    else if(l_speed < 0)
    {
        //左电机后退 IN0=1 IN1=0
        HAL_GPIO_WritePin(LEFT_WHELL_IN0_GPIO_Port, LEFT_WHELL_IN0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_WHELL_IN1_GPIO_Port, LEFT_WHELL_IN1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -l_speed);
    }
    else
    {
        //刹车 IN0=0 IN1=0
        HAL_GPIO_WritePin(LEFT_WHELL_IN0_GPIO_Port, LEFT_WHELL_IN0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_WHELL_IN1_GPIO_Port, LEFT_WHELL_IN1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }

    // ==========右电机 PB2 PB3==========
    if(r_speed > 0)
    {
        //右电机前进 IN0=0 IN1=1
        HAL_GPIO_WritePin(RIGHT_WHELL_IN0_GPIO_Port, RIGHT_WHELL_IN0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_WHELL_IN1_GPIO_Port, RIGHT_WHELL_IN1_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, r_speed);
    }
    else if(r_speed < 0)
    {
        //右电机后退 IN0=1 IN1=0
        HAL_GPIO_WritePin(RIGHT_WHELL_IN0_GPIO_Port, RIGHT_WHELL_IN0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_WHELL_IN1_GPIO_Port, RIGHT_WHELL_IN1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -r_speed);
    }
    else
    {
        //刹车
        HAL_GPIO_WritePin(RIGHT_WHELL_IN0_GPIO_Port, RIGHT_WHELL_IN0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_WHELL_IN1_GPIO_Port, RIGHT_WHELL_IN1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

//整车前进 speed：0‑999
void TB6612_Car_Forward(uint16_t speed)
{
    TB6612_SetMotor(speed, speed);
}

//整车后退
void TB6612_Car_Backward(uint16_t speed)
{
    TB6612_SetMotor(-(int16_t)speed, -(int16_t)speed);
}

//原地左转：左轮停，右轮前进
void TB6612_Car_TurnLeft(uint16_t speed)
{
    TB6612_SetMotor(0, speed);
}

//原地右转：右轮停，左轮前进
void TB6612_Car_TurnRight(uint16_t speed)
{
    TB6612_SetMotor(speed, 0);
}

//原地掉头左转：左轮后退，右轮前进
void TB6612_Car_SpinLeft(uint16_t speed)
{
    TB6612_SetMotor(-(int16_t)speed, speed);
}

//原地掉头右转：左轮前进，右轮后退
void TB6612_Car_SpinRight(uint16_t speed)
{
    TB6612_SetMotor(speed, -(int16_t)speed);
}

void TB6612_Car_Stop(void)
{
    TB6612_SetMotor(0, 0);
}
