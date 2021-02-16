/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "prototype.h"
//#include "meme.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define PD_BUFFER_SIZE (FRAME_SIZE)
#define BUFFER_SIZE (50) // For now let n = j

#define ATTACK_TIME (70)
#define AD_BUFFER_SIZE (BUFFER_SIZE + ATTACK_TIME)

#define DMA_BUFFER_SIZE ((BUFFER_SIZE*2U) * 2U) // First double due to being in stereo, second due to being "two half-buffers"
#define DMA_BUFFER_SIZE_BYTES (DMA_BUFFER_SIZE * 4U)


typedef enum {
	WAIT,
	FILL_PD_BUFFER
} action_t;

typedef struct {
	action_t action;
	uint8_t cur_midi;
	uint8_t playing;
	int32_t PD_buffer[PD_BUFFER_SIZE];
	size_t PD_pointer;
	int32_t AD_buffer[AD_BUFFER_SIZE];
} state_t;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void fillProcessingBuffer();
void sendMIDI(uint8_t*msg, size_t size);
void MIDIon(uint8_t channel, uint8_t note, uint8_t velocity);
void MIDIoff(uint8_t channel, uint8_t note, uint8_t velocity);
uint8_t ProcessActiveBuffer();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_RESET_Pin GPIO_PIN_11
#define ADC_RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
