/*
 * drv_usart2_poll.c
 *
 *  Created on: 2026年9月3日
 *      Author: 27059
 */

#include "stm32g0xx_hal.h"
#include "drv_usart2_poll.h"
#include "stdio.h"
#include "string.h"
#include "drv_TB6612FNG.h"


/* 帧定义 */
#define FRAME_LEN        7U
#define FRAME_HEAD0      0x5AU
#define FRAME_HEAD1      0xA5U
#define FRAME_TAIL       0xFFU

/* 接收缓冲区与状态 */
typedef struct
{
    uint16_t data1;
    uint16_t data2;
    uint8_t  frame_ok;
}CarFrame_t;

static uint8_t rx_buf[FRAME_LEN];
static uint8_t rx_idx = 0U;
uint8_t rx_ch;
CarFrame_t car_rx;
static char log_buf[128];
extern UART_HandleTypeDef huart2;

/**
 * @brief  串口打印辅助函数，轮询发送
 */
void uart_print(UART_HandleTypeDef *huart, char *str)
{
    HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), 100);
}


/**
  * @brief UART接收完成回调函数，中断里执行！
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        //rx_ch 就是本次中断收到的字节
        uint8_t ch = rx_ch;

        //帧解析逻辑，和之前轮询逻辑一致
        if(rx_idx >= FRAME_LEN)
        {
            rx_idx = 0;
        }
        rx_buf[rx_idx] = ch;

        if(rx_idx == 0)
        {
            if(ch != FRAME_HEAD0)
            {
                rx_idx =0;
                //重新开启中断接收，必须！
                HAL_UART_Receive_IT(&huart2, &rx_ch,1);
                return;
            }
        }
        else if(rx_idx ==1)
        {
            if(ch != FRAME_HEAD1)
            {
                rx_idx =0;
                HAL_UART_Receive_IT(&huart2, &rx_ch,1);
                return;
            }
        }

        rx_idx++;

        if(rx_idx >= FRAME_LEN)
        {
            rx_idx = 0;
            if(rx_buf[6] == FRAME_TAIL)
            {
                car_rx.data1 = ((uint16_t)rx_buf[2] <<8) | rx_buf[3];
                car_rx.data2 = ((uint16_t)rx_buf[4] <<8) | rx_buf[5];
                car_rx.frame_ok = 1U;
            }
        }

        //⚠️非常关键！再次调用，继续等待下一个字节中断
        HAL_UART_Receive_IT(&huart2, &rx_ch, 1);
    }
}


/**
 * @brief  USART2轮询解析一帧，放在while(1)循环内调用
 * @param  husart2: &huart2
 * @param  out: 解析输出结果
 */
void USART2_Poll_Parse(UART_HandleTypeDef *huart2)
{
    int16_t l_pwm,r_pwm;

	if(car_rx.frame_ok == 1U)
	{
		// ==========打印原始报文与解析结果==========
		sprintf(log_buf,
		        "RX Frame: %02X %02X %02X %02X %02X %02X %02X \r\n",
		        rx_buf[0],rx_buf[1],rx_buf[2],rx_buf[3],rx_buf[4],rx_buf[5],rx_buf[6]);
		        uart_print(huart2, log_buf);

		//这里处理速度，电机控制；注意：不要在while1里面sprintf打印！中断内禁止printf/sprintf！
	    l_pwm = ((int16_t)rx_buf[2]<<8) | rx_buf[3];
	    r_pwm = ((int16_t)rx_buf[4]<<8) | rx_buf[5];

	    TB6612_SetMotor(l_pwm,r_pwm);

		car_rx.frame_ok = 0U;
	}
}
