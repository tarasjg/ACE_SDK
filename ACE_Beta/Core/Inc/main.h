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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t data[250][27];
//extern volatile uint8_t flag;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_DRDY_Pin GPIO_PIN_0
#define ACC_DRDY_GPIO_Port GPIOA
#define ACC_DRDY_EXTI_IRQn EXTI0_IRQn
#define ACC_CS_Pin GPIO_PIN_4
#define ACC_CS_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define ADS_PWDN_Pin GPIO_PIN_12
#define ADS_PWDN_GPIO_Port GPIOC
#define APWR_EN_Pin GPIO_PIN_2
#define APWR_EN_GPIO_Port GPIOD
#define ADS_MISO_Pin GPIO_PIN_4
#define ADS_MISO_GPIO_Port GPIOB
#define ADS_MOSI_Pin GPIO_PIN_5
#define ADS_MOSI_GPIO_Port GPIOB
#define ADS_CS_Pin GPIO_PIN_6
#define ADS_CS_GPIO_Port GPIOB
#define ADS_DRDY_Pin GPIO_PIN_7
#define ADS_DRDY_GPIO_Port GPIOB
#define ADS_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define ADS_START_Pin GPIO_PIN_8
#define ADS_START_GPIO_Port GPIOB
#define ADS_RST_Pin GPIO_PIN_9
#define ADS_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MEM_FULL 0xF0  //indicates memory is full and blocks further memory writes
#define AFE_DRDY 0x08  //indicates that the front end has signaled that a sample is ready
#define ACC_DRDY 0x04  //indicates that the accelerometer has signaled that a sample buffer is ready
#define SYS_DUMP 0x02  //indicates that the system has received a magic word asking for a flash memory dump
#define SYS_CLR  0x01  //indicates that the system has received a magic word asking to erase the flash

#define ACC_DRDY_AND_AFE_DRDY_AND_MEM_AVAIL 0xC
#define AFE_DRDY_AND_MEM_AVAIL 0x08
#define ACC_DRDY_AND_MEM_AVAIL 0x04
#define SYS_DUMP_AND_MEM_FULL  0xF2
#define SYS_CLR_AND_MEM_FULL   0xF1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
