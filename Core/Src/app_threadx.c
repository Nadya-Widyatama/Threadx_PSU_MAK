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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024
#define BUFFER_SIZE 10
#define BatteryCapacity1 2.0 //Measurement in units of Ah
#define BatteryCapacity2 4.0 //Measurement in units of Ah
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t thread_Setone[THREAD_STACK_SIZE];
uint8_t thread_Settwo[THREAD_STACK_SIZE];
uint8_t thread_Setthree[THREAD_STACK_SIZE];
uint8_t thread_Setfour[THREAD_STACK_SIZE];

TX_THREAD setone;
TX_THREAD settwo;
TX_THREAD setthree;
TX_THREAD setfour;

void Beep_Beep(uint8_t cycle, uint16_t delay1, uint16_t delay2);
void write_value(float value, uint32_t address);
void ReadData(uint32_t address, uint32_t length);

uint32_t NowMillis1, BeforeMillis1,NowMillis2, BeforeMillis2;
uint32_t adcBuffer[4];
float temperature,voltage1,current1,voltage2,current2,ConsumptionEnergy1,ConsumptionEnergy2;
float read_data_float, write_value_float,CurrentFiltered1,batterypercentage1,CurrentFiltered2,batterypercentage2;
int before = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID ReadADC_voltage_current(ULONG initial_input);
VOID Setup(ULONG initial_input);
VOID Set_LED(ULONG initial_input);
VOID Transmit(ULONG initial_input);
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
	tx_thread_create(&settwo, "Settwo", Setup, 0, thread_Settwo, THREAD_STACK_SIZE, 13, 13, 1, TX_AUTO_START);
	tx_thread_create(&setthree, "Setthree", Set_LED, 0, thread_Setthree, THREAD_STACK_SIZE, 12, 12, 1, TX_AUTO_START);
	tx_thread_create(&setfour, "Setfour", Transmit, 0, thread_Setfour, THREAD_STACK_SIZE, 12, 12, 1, TX_AUTO_START);
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

    	//Current batt1 Consumption Algorithm
    	NowMillis1 = tx_time_get();
    	if (NowMillis1 - BeforeMillis1 >= 1000){
    		CurrentFiltered1 = 0.2 * current1 + 0.8 * CurrentFiltered1;
    		ConsumptionEnergy1 += (CurrentFiltered1 / 3600);
    		batterypercentage1 = ((BatteryCapacity1 - ConsumptionEnergy1) / BatteryCapacity1) *100;
    		BeforeMillis1 = NowMillis1;
    	}

    	//Current batt2 Consumption Algorithm
    	NowMillis2 = tx_time_get();
    	if (NowMillis2 - BeforeMillis2 >= 1000){
    		CurrentFiltered2 = 0.2 * current2 + 0.8 * CurrentFiltered2;
    		ConsumptionEnergy2 += (CurrentFiltered2 / 3600);
    		batterypercentage2 = ((BatteryCapacity2 - ConsumptionEnergy2) / BatteryCapacity2) *100;
    		BeforeMillis2 = NowMillis2;
//    		printf("Voltage1 : %.2f |", voltage1);
//    		printf("current1 : %.4f |", current1);
//    		printf("Voltage2 : %.2f |", voltage2);
//    		printf("current2 : %.4f |", current2);
//    		printf("Consumption : %.4f Ah |", ConsumptionEnergy1);
//    		printf("percentage : %d %%\n", (int)round(batterypercentage1));
    	}
    }
}

void Setup(ULONG initial_input) {
	Beep_Beep(2,100,50);
    while(1) {
    	tx_thread_sleep(TX_WAIT_FOREVER);
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

void Transmit(ULONG initial_input) {
	char buffer[128];
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	while(1) {
		int len = snprintf(buffer, sizeof(buffer),
				"Voltage1 : %.2f | current1 : %.4f | Voltage2 : %.2f | current2 : %.4f | Consumption : %.4f Ah | percentage : %d %%\n",
				voltage1, current1, voltage2, current2, ConsumptionEnergy1, (int)round(batterypercentage1));
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
		tx_thread_sleep(1000);
	}
}

//Function to produce a beep sound on the buzzer
void Beep_Beep(uint8_t cycle, uint16_t delay1, uint16_t delay2) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	for (int i = 0; i < cycle; i++) {
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3000);
		tx_thread_sleep(delay1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		tx_thread_sleep(50);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
		tx_thread_sleep(delay2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		tx_thread_sleep(50);
	}
}

// Function to erase and write values to flash memory
void write_value(float value, uint32_t address) {
	uint8_t write_enable_cmd = 0x06;
    uint8_t data[sizeof(value)];
    memcpy(data, &value, sizeof(value));

    //Erase Sector Command (4 KB)
    uint8_t erase_cmd[4];
    erase_cmd[0] = 0x20;
    erase_cmd[1] = (address >> 16) & 0xFF;
    erase_cmd[2] = (address >> 8) & 0xFF;
    erase_cmd[3] = address & 0xFF;

    //Write Enable Command
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, erase_cmd, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(100);

    // command Write Data
    uint8_t write_cmd[4];
    write_cmd[0] = 0x02;
    write_cmd[1] = (address >> 16) & 0xFF;
    write_cmd[2] = (address >> 8) & 0xFF;
    write_cmd[3] = address & 0xFF;

    //Write Enable Command
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, sizeof(data), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(50);
}

//Function to read data from flash memory
void ReadData(uint32_t address, uint32_t length) {
    uint8_t cmd[4];
    cmd[0] = 0x03;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    uint8_t data[length];
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    memcpy(&read_data_float, data, sizeof(read_data_float));
}
/* USER CODE END 1 */
