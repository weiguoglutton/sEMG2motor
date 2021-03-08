/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define EMG_ADC 0x00000003     //adc通道设置，通过控制寄存器转换sEMG与power voltage之间的通道
#define PWR_ADC 0x00000200			//

#define ADC_TIM_PERIOD 199			//设置adc对应的timer采样间隔,72mhz/72/(ADC_TIM_PERIOD+1)
#define LED_TIM_PERIOD 999				//设置led对应timer时基
#define MOTOR_A_TIM_PERIOD 999		//设置电机pwm频率
#define KEY_TIM_PERIOD 1999				//设置按键时基
	
#define ADC_LEN 1000							//设置adc 长度，ADC_LEN/channel=采样数量，72m/(ADC_TIM_PERIOD+1)/ADC_LEN=1s采样次数
#define S_CHANNEL__NUM 2U					//sEMG通道数量
#define CHA 0U										//设置通道序列
#define CHB 1U
#define CH_ALL 0xFF								
#define CH_DIS 0xFF 
#define SUBDIVIDE 0xFF						//adc value分辨率
#define S_VALUE_CENTERLINE_CHA 30	//设置不同通道sEMG默认中心线基准，此能级对应的是SUBDIVIDE格式化后的数量级
#define S_VALUE_CENTERLINE_CHB 10
#define S_VALUE_TOP_CHA 200					//设置sEMG阈值，最高不超过SUBDIVIDE
#define S_VALUE_TOP_CHB 200
#define SET_S_VALUE_CENTERLINE_CHA_MIN 5 //设置状态下对应通道读取的能级下限
#define SET_S_VALUE_CENTERLINE_CHB_MIN 5	//
#define SET_S_VALUE_CENTERLINE_CHA_MAX 80 //设置状态下对应通道读取的能级上限
#define SET_S_VALUE_CENTERLINE_CHB_MAX 80
#define COMPARE_TH 10U										//sEMG通道之间使能差低于这个值则忽略
#define S_VALUE_COUNT_TH 0U								//过滤样本最低数量。忽略低于这个数量的样本
#define S_VALUE_SET_CYCLE 8U							//设置状态下的采样循环次数
#define S_VALUE_SET_TH	2U;								//设置状态下的能级偏移量

#define SPEED_SUBDIVIDE 100									//电机pwm占空比分辨率
#define MOTOR_A_DIR_A_EN MOTOR_A_TIM.Instance->CCER=0x10 	//开关pwm通道
#define MOTOR_A_DIR_B_EN MOTOR_A_TIM.Instance->CCER=0x01

//#define MOTOR_A_DIR_A_EN MOTOR_A_TIM.Instance->CCER=0x00  	//替换以上设置，关闭电机通道用于仿真测试
//#define MOTOR_A_DIR_B_EN MOTOR_A_TIM.Instance->CCER=0x00
#define MOTOR_A_DIS MOTOR_A_TIM.Instance->CCER=0U
#define MOTOR_A_CCRA MOTOR_A_TIM.Instance->CCR1		//设置pwm占空比
#define MOTOR_A_CCRB MOTOR_A_TIM.Instance->CCR2  //TIM3->CCR1 TIM3->CCR2

#define LED_SUBDIVIDE 100										//设置led pwm分辨率
#define LED_kEEP_ON 100
#define LED_CENTERLINE 20

#define KEY_SHORT 16						
#define KEY_LONG 750
#define KEY_TIME_OUT 2000
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_A_TIM htim3
#define ADC_TIM htim1
#define LED_TIM htim14
#define KEY_TIM htim16
#define LED_CHA_Pin GPIO_PIN_2
#define LED_CHA_GPIO_Port GPIOA
#define KEY_B_Pin GPIO_PIN_3
#define KEY_B_GPIO_Port GPIOA
#define KEY_B_EXTI_IRQn EXTI2_3_IRQn
#define LED_CHB_Pin GPIO_PIN_4
#define LED_CHB_GPIO_Port GPIOA
#define KEY_A_Pin GPIO_PIN_5
#define KEY_A_GPIO_Port GPIOA
#define KEY_A_EXTI_IRQn EXTI4_15_IRQn
#define null1_Pin GPIO_PIN_9
#define null1_GPIO_Port GPIOA
#define null0_Pin GPIO_PIN_10
#define null0_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
