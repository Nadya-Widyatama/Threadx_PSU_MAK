/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024
#define BUFFER_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t thread_Setone[THREAD_STACK_SIZE];
uint8_t thread_Settwo[THREAD_STACK_SIZE];
uint8_t thread_Setthree[THREAD_STACK_SIZE];

TX_THREAD setone;
TX_THREAD settwo;
TX_THREAD setthree;

uint32_t NowMillis, SebelumMillis;
uint32_t adcBuffer[4];
float temperature,voltage1,current1,voltage2,current2,konsumsiEnergi;
float read_data_float, write_value_float,arusFiltered;
int before = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID ReadADC_voltage_current(ULONG initial_input);
VOID Set_print(ULONG initial_input);
VOID Set_LED(ULONG initial_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
	(void)byte_pool;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
	tx_thread_create(&setone, "Setone", ReadADC_voltage_current, 0, thread_Setone, THREAD_STACK_SIZE, 14, 14, 1, TX_AUTO_START);
	tx_thread_create(&settwo, "Settwo", Set_print, 0, thread_Settwo, THREAD_STACK_SIZE, 13, 13, 1, TX_AUTO_START);
	tx_thread_create(&setthree, "Setthree", Set_LED, 0, thread_Setthree, THREAD_STACK_SIZE, 12, 12, 1, TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
void ReadADC_voltage_current(ULONG initial_input) {
    while(1	) {

    	uint32_t sumADC_voltage1 = 0, sumADC_current1 = 0,sumADC_voltage2 = 0, sumADC_current2 = 0;
    	uint16_t value_voltage1, value_current1, value_voltage2, value_current2;
    	float voltage_current1,voltage_current2;

    	for (int i = 0; i < 500; i++) {
    		HAL_ADC_Start_DMA(&hadc1, adcBuffer, 4);
    		sumADC_voltage1 += adcBuffer[0];
    		sumADC_voltage2 += adcBuffer[1];
    		sumADC_current1 += adcBuffer[2];
    		sumADC_current2 += adcBuffer[3];
    		HAL_ADC_Stop_DMA(&hadc1);
    	}

    	value_voltage1 = ((sumADC_voltage1 / 500) - 60) * 3882 / (3942 - 60);
    	value_current1 = ((sumADC_current1 / 500) - 60) * 4035 / (4095 - 60);
    	value_voltage2 = ((sumADC_voltage2 / 500) - 59) * 3886 / (3994 - 59);
    	value_current2 = ((sumADC_current2 / 500) - 57) * 4038 / (4095 - 57);

    	voltage1 = (value_voltage1 * 14.6) / 3882;
    	voltage_current1 = (value_current1 * 3.31) / 4095;
    	current1 = fabs((voltage_current1 - 2.5) / 0.097);

    	voltage2 = (value_voltage2 * 14.6) / 3836;
    	voltage_current2 = (value_current2 * 3.31) / 4095;
    	current2 = fabs((voltage_current2 - 2.5) / 0.098);

    	//Konsumsi Arus Algoritma
    	NowMillis = tx_time_get();
    	if (NowMillis - SebelumMillis >= 1000){
    		arusFiltered = 0.2 * current1 + 0.8 * arusFiltered;
    		konsumsiEnergi += (arusFiltered / 3600);
    		SebelumMillis = NowMillis;
    	}
    	printf("Voltage1 : %.2f |", voltage1);
    	printf("current1 : %.4f |", current1);
    	printf("Voltage2 : %.2f |", voltage2);
    	printf("current2 : %.4f \n", current2);
    }
}

void Set_print(ULONG initial_input) {
	int counter = 1;
    while(1) {
    	printf("Hai %d\n", counter);
    	counter++;
    	tx_thread_sleep(1000);
    }
}

void Set_LED(ULONG initial_input) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	while(1) {
		for(int i = 0; i < 4095; i++){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
			tx_thread_sleep(1);
		}
		for(int i = 4095; i > 0; i--){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
			tx_thread_sleep(1);
		}
	}
}

/* USER CODE END 1 */
