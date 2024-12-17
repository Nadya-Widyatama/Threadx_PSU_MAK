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
#include "DS18B20.h"
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
#define THREAD_STACK_SIZE 2048
#define BUFFER_SIZE 10
#define BatteryCapacity1 2.0 	//Measurement in units of Ah
#define BatteryCapacity2 3.31 	//Measurement in units of Ah
#define SoC_End_Discharge 20;
#define SoC_End_Charge 80

#define DS18B20_PIN GPIO_PIN_5
#define DS18B20_PORT GPIOA
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
uint8_t thread_Setfive[THREAD_STACK_SIZE];
uint8_t thread_Setsix[THREAD_STACK_SIZE];
uint8_t thread_Setseven[THREAD_STACK_SIZE];
uint8_t thread_Seteight[THREAD_STACK_SIZE];

TX_THREAD setone;
TX_THREAD settwo;
TX_THREAD setthree;
TX_THREAD setfour;
TX_THREAD setfive;
TX_THREAD setsix;
TX_THREAD setseven;
TX_THREAD seteight;

void Beep_Beep(uint8_t cycle, uint16_t delay1, uint16_t delay2);
void write_value(void *value, uint32_t address, size_t size);
void ReadData(void *buffer, uint32_t address, size_t size);
const char* get_status_string(uint8_t status);

uint32_t NowMillis1, BeforeMillis1,NowMillis2, BeforeMillis2;
float temperature,voltage1,current1,voltage2,current2;
float write_value_float,batterypercentage1,batterypercentage2;
float AH_Restored1, AH_Consumed1, AH_Restored2, AH_Consumed2;
int before = 0;
uint8_t SoH1, SoH2;

// Variabel global untuk buffer dan flag
//volatile uint8_t requestBuffer[1] = {0};
//volatile uint8_t dataReady = 0;

//float temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID ADC_Reading(ULONG initial_input);
VOID Setup(ULONG initial_input);
VOID Set_LED(ULONG initial_input);
VOID Transmit_Data(ULONG initial_input);
VOID Power_Consumption(ULONG initial_input);
VOID Temperature_Reading(ULONG initial_input);
VOID Memory_Management(ULONG initial_input);
VOID SoH_Management(ULONG initial_input);

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
	tx_thread_create(&setone, "Setone", ADC_Reading, 0, thread_Setone, THREAD_STACK_SIZE, 4, 4, 1, TX_AUTO_START);
	tx_thread_create(&settwo, "Settwo", Setup, 0, thread_Settwo, THREAD_STACK_SIZE, 6, 6, 1, TX_AUTO_START);
	tx_thread_create(&setthree, "Setthree", Set_LED, 0, thread_Setthree, THREAD_STACK_SIZE, 6, 6, 1, TX_AUTO_START);
	tx_thread_create(&setfour, "Setfour", Transmit_Data, 0, thread_Setfour, THREAD_STACK_SIZE, 6, 6, 1, TX_AUTO_START);
	tx_thread_create(&setfive, "Setfive", Power_Consumption, 0, thread_Setfive, THREAD_STACK_SIZE, 7, 7, 1, TX_AUTO_START);
	tx_thread_create(&setsix, "Setsix", Temperature_Reading, 0, thread_Setsix, THREAD_STACK_SIZE, 4, 4, 1, TX_AUTO_START);
	//tx_thread_create(&setseven, "Setseven", Memory_Management, 0, thread_Setseven, THREAD_STACK_SIZE, 5, 5, 1, TX_AUTO_START);
	tx_thread_create(&seteight, "Seteight", SoH_Management, 0, thread_Seteight, THREAD_STACK_SIZE, 5, 5, 1, TX_AUTO_START);
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
void ADC_Reading(ULONG initial_input) {
	uint16_t value_voltage1, value_current1, value_voltage2, value_current2;
	uint32_t sumADC_voltage1, sumADC_current1, sumADC_voltage2, sumADC_current2;
	float voltage_current1,voltage_current2;
	uint32_t adcBuffer[4];

    while(1) {
    	sumADC_voltage1 = 0;
    	sumADC_current1 = 0;
    	sumADC_voltage2 = 0;
    	sumADC_current2 = 0;

    	HAL_ADC_Start_DMA(&hadc1, adcBuffer, 4);
    	for (int i = 0; i < 50; i++) {
    		sumADC_voltage1 += adcBuffer[0];
    		sumADC_voltage2 += adcBuffer[1];
    		sumADC_current1 += adcBuffer[2];
    		sumADC_current2 += adcBuffer[3];
    		tx_thread_sleep(10);
    	}
    	HAL_ADC_Stop_DMA(&hadc1);

    	uint32_t avg_voltage1 = sumADC_voltage1 / 50;
    	uint32_t avg_current1 = sumADC_current1 / 50;
    	uint32_t avg_voltage2 = sumADC_voltage2 / 50;
    	uint32_t avg_current2 = sumADC_current2 / 50;

    	value_voltage1 = ((avg_voltage1 - 64) * 1023) / (3953 - 64);
    	value_current1 = ((avg_current1 - 60) * 1023) / (4095 - 60);
    	value_voltage2 = ((avg_voltage2 - 67) * 1023) / (3960 - 67);
    	value_current2 = ((avg_current2 - 61) * 1023) / (4095 - 61);

    	if(value_voltage1 > 3882){
    		value_voltage1 = 0;
    	}
    	if(value_voltage2 > 3886){
    		value_voltage2 = 0;
    	}

    	voltage1 = (value_voltage1 * 14.6) / 1023;
    	voltage_current1 = (value_current1 * 3.253) / 1023;
    	current1 = ((voltage_current1 - 2.5185) / 0.067125);

    	voltage2 = (value_voltage2 * 14.6) / 1023;
    	voltage_current2 = (value_current2 * 3.253) / 1023;
    	current2 = ((voltage_current2 - 2.5121) / 0.067125); //0.0665

    	if(voltage1 == 0){
    		current1 = 0.0;
    	}
    	if(voltage2 == 0){
    		current2 = 0.0;
    	}
//    	if((current1 <= -10 && current1 > -15) || (current2 <= -10 && current2 > -15)){
//    		Beep_Beep(1,100,100);
//    	}
//    	else if(current1 <= -15 || current2 <= -15){
//    		Beep_Beep(2,50,50);
//    	}

//    	printf("value_voltage1 :    %d | ", avg_voltage1);
//    	printf("value_voltage2 : %d | ", avg_voltage2);
//    	printf("voltage1 : %.2f | ", voltage1);
//    	printf("voltage2 : %.2f | ", voltage2);
//    	printf("Current1 : %.3f | ", current1);
//    	printf("Current2 : %.3f | \n", current2);
    }
}

void Power_Consumption(ULONG initial_input) {
	float CurrentFiltered1,CurrentFiltered2;
	while(1) {
		// Current batt1 Consumption Algorithm
		NowMillis1 = tx_time_get();
		if (NowMillis1 - BeforeMillis1 >= 1000) {
			CurrentFiltered1 = 0.2 * current1 + 0.8 * CurrentFiltered1;
			if (current1 < 0) {
				AH_Consumed1 -= (CurrentFiltered1 / 3600);
				status1 = "Discharge";
			}
			else {
				AH_Restored1 += (CurrentFiltered1 / 3600);
				status1 = "Charger";
			}
			batterypercentage1 = ((BatteryCapacity1 - AH_Consumed1 + AH_Restored1) / BatteryCapacity1) * 100;
			if (batterypercentage1 > 100.0) {
				batterypercentage1 = 100.0;
			}
			else if(batterypercentage1 < 0.0) {
				batterypercentage1 = 0;
			}
			BeforeMillis1 = NowMillis1;
		}

		// Current batt2 Consumption Algorithm
		NowMillis2 = tx_time_get();
		if (NowMillis2 - BeforeMillis2 >= 1000) {
			CurrentFiltered2 = 0.2 * current2 + 0.8 * CurrentFiltered2;
			if (current2 < 0) {
				AH_Consumed2 -= (CurrentFiltered2 / 3600);
				status2  = "Discharge";
			}
			else {
				AH_Restored2 += (CurrentFiltered2 / 3600);
				status2 = "Charger";
			}
			batterypercentage2 = ((BatteryCapacity2 - AH_Consumed2 + AH_Restored2) / BatteryCapacity2) * 100;
			if (batterypercentage2 > 100.0) {
				batterypercentage2 = 100.0;
			}
			else if(batterypercentage2 < 0.0) {
				batterypercentage2 = 0;
			}
			BeforeMillis2 = NowMillis2;
		}

//	printf("voltage1 : %.2f volt| ", voltage1);
//	printf("Current1 : %.3f A | ", current1);
//	printf("AH_Consumed1 : %.3f Ah |", AH_Consumed1);
//	printf("voltage2 : %.2f volt | ", voltage2);
//	printf("Current2 : %.3f A | ", current2);
//	printf("AH_Consumed2 : %.3f Ah\n|", AH_Consumed2);

	//printf("AH_Restored1 : %.3f |", AH_Restored1);
	//printf("batterypercentage1: %.2f |", batterypercentage2);
	//printf("status : %s \n", status2);
	tx_thread_sleep(1000);
	}
}


// Key threads for SoH management
void SoH_Management(ULONG initial_input) {
    static float voltage_no_load1 = 0.0f;
    static float voltage_with_load1 = 0.0f;
    static float voltage_no_load2 = 0.0f;
    static float voltage_with_load2 = 0.0f;
    float voltage_drop1, voltage_drop2 = 0.0f;

    // Momory address for storing status
    uint32_t address1 = 0x000012;
    uint32_t address2 = 0x000016;

    // start the status by reading form flash memory

    ReadData(&SoH1, address1, sizeof(SoH1));
    ReadData(&SoH2, address2, sizeof(SoH2));

    // if flash is empty, set default to Healthy
    if (SoH1 > 3) SoH1 = 0;
    if (SoH2 > 3) SoH2 = 0;

    uint8_t previousSoH1 = SoH1;
    uint8_t previousSoH2 = SoH2;

    while (1) {
        if (current1 < 0.0f && batterypercentage1 >= 80) {
            if (fabs(current1) < 0.3f) {
                voltage_no_load1 = voltage1;
                voltage_with_load1 = 0.0f;
                voltage_drop1 = 0.0f;
            } else {
                voltage_with_load1 = voltage1;
                voltage_drop1 = voltage_no_load1 - voltage_with_load1;
            }

            if (current1 < -10.0f && voltage_drop1 >= 2.0f && SoH1 != 3) {
                SoH1 = 3; // Weak
            } else if (current1 < -5.0f && current1 >= -10.0f && voltage_drop1 < 2.0f && voltage_drop1 >= 1.0f && SoH1 != 2 && SoH1 != 3) {
                SoH1 = 2; // Moderate
            } else if (current1 < -1.0f && current1 >= -5.0f && voltage_drop1 < 1.0f && voltage_drop1 >= 0.5f && SoH1 != 1 && SoH1 != 2 && SoH1 != 3) {
                SoH1 = 1; // Good
            } else if (voltage_drop1 < 0.5f && SoH1 == 0) {
                SoH1 = 0; // Healthy
            }
        }

        if (current2 < 0.0f && batterypercentage2 >= 80) {
            if (fabs(current2) < 0.3f) {
                voltage_no_load2 = voltage2;
                voltage_with_load2 = 0.0f;
                voltage_drop2 = 0.0f;
            } else {
                voltage_with_load2 = voltage2;
                voltage_drop2 = voltage_no_load2 - voltage_with_load2;
            }

            if (current2 < -10.0f && voltage_drop2 >= 2.0f && SoH2 != 3) {
                SoH2 = 3; // Weak
            } else if (current2 < -5.0f && current2 >= -10.0f && voltage_drop2 < 2.0f && voltage_drop2 >= 1.0f && SoH2 != 2 && SoH2 != 3) {
                SoH2 = 2; // Moderate
            } else if (current2 < -1.0f && current2 >= -5.0f && voltage_drop2 < 1.0f && voltage_drop2 >= 0.5f && SoH2 != 1 && SoH2 != 2 && SoH2 != 3) {
                SoH2 = 1; // Good
            } else if (voltage_drop2 < 0.5f && SoH2 == 0) {
                SoH2 = 0; // Healthy
            }
        }

        // Save status to memory only if there is a change
        if (SoH1 != previousSoH1) {
        	write_value(&SoH1, address1, sizeof(SoH1));
        	Beep_Beep(5,50,50);
        	previousSoH1 = SoH1;
        }

        if (SoH2 != previousSoH2) {
        	write_value(&SoH2, address2, sizeof(SoH2));
        	Beep_Beep(5,50,50);
        	previousSoH2 = SoH2;
        }

//        printf("Baterai 1: %s\n", get_status_string(SoH1));
//        printf("Current1: %.2f A | Voltage1 No Load: %.2f V | Voltage1 With Load: %.2f V | Voltage Drop1: %.2f V\n",
//               current1, voltage_no_load1, voltage_with_load1, voltage_drop1);
//
//        printf("Baterai 2: %s\n", get_status_string(SoH2));
//        printf("Current2: %.2f A | Voltage2 No Load: %.2f V | Voltage2 With Load: %.2f V | Voltage Drop2: %.2f V\n",
//               current2, voltage_no_load2, voltage_with_load2, voltage_drop2);

        tx_thread_sleep(1000);
    }
}

void Setup(ULONG initial_input) {
	Beep_Beep(2,100,50);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
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



void Transmit_Data(ULONG initial_input) {
    uint8_t requestBuffer[0];
    char buffer[128];
    while (1) {
        // Menerima request dari UART
        HAL_HalfDuplex_EnableReceiver(&huart2);
        HAL_UART_Receive(&huart2, requestBuffer, 1, 1000);

        // Group first : voltage1, current1, AH_Consumed1, batterypercentage1, status1
        if (requestBuffer[0] == 0x55) {
        	tx_thread_sleep(30);
            int len = snprintf(buffer, sizeof(buffer),
                "Volt1: %.2f V | Curr1: %.2f A | Cons1: %.4f Ah | Batt1: %d%% | Stat1: %s\n",
                voltage1, current1, AH_Consumed1, (int)round(batterypercentage1), status1);
            HAL_HalfDuplex_EnableTransmitter(&huart2);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        }

        // Group second : voltage2, current2, AH_Consumed2, batterypercentage2, status2
        else if (requestBuffer[0] == 0x66) {
        	tx_thread_sleep(30);
            int len = snprintf(buffer, sizeof(buffer),
                "Volt2: %.2f V | Curr2: %.2f A | Cons2: %.4f Ah | Batt2: %d%% | Stat2: %s\n",
                voltage2, current2, AH_Consumed2, (int)round(batterypercentage2), status2);
            HAL_HalfDuplex_EnableTransmitter(&huart2);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        }

        // Group third : temperature, SoH batt 1 and 2
        else if (requestBuffer[0] == 0x77) {
        	tx_thread_sleep(30);
            int len = snprintf(buffer, sizeof(buffer),
            		"Temp: %.2f C | SoH1 : %s | SoH2 : %s\n",
					temperature, get_status_string(SoH1), get_status_string(SoH2));
            HAL_HalfDuplex_EnableTransmitter(&huart2);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        }
        tx_thread_sleep(50);
    }
}

void Memory_Management (ULONG initial_input) {
	uint32_t address = 0x000000;
	char nilai[4] = "hai";
	write_value(nilai, address, sizeof(float));

	while(1) {
		ReadData(nilai, address, sizeof(nilai));
		printf("Read Data: %s \n", nilai);
		//printf("Temp: %.2f\n", temperature);
		Beep_Beep(1,50,50);
		tx_thread_sleep(3000);
	}
}

//Function to read temperature from DS18B20
void Temperature_Reading(ULONG initial_input){
	while(1){
		temperature = DS18B20_GetTemp();
		tx_thread_sleep(1000);
	}
}

//Function Group Zone
//Function to produce a beep sound on the buzzer

// Fungsi untuk mengubah angka menjadi status SoH
const char* get_status_string(uint8_t status) {
    switch (status) {
        case 0: return "Healthy";
        case 1: return "Good";
        case 2: return "Moderate";
        case 3: return "Weak";
        default: return "Unknown";
    }
}

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

//Function to erase and write values to flash memory
void write_value(void *value, uint32_t address, size_t size) {
    uint8_t write_enable_cmd = 0x06;
    uint8_t data[size];
    memcpy(data, value, size);

    // Erase Sector Function (4 KB)
    uint8_t erase_cmd[4];
    erase_cmd[0] = 0x20;
    erase_cmd[1] = (address >> 16) & 0xFF;
    erase_cmd[2] = (address >> 8) & 0xFF;
    erase_cmd[3] = address & 0xFF;

    // Write Enable Function
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, erase_cmd, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(100);

    // Command Write Data
    uint8_t write_cmd[4];
    write_cmd[0] = 0x02;
    write_cmd[1] = (address >> 16) & 0xFF;
    write_cmd[2] = (address >> 8) & 0xFF;
    write_cmd[3] = address & 0xFF;

    // Perintah Write Enable
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(50);
}


void ReadData(void *buffer, uint32_t address, size_t size) {
    uint8_t read_cmd[4];
    read_cmd[0] = 0x03; // Command untuk membaca
    read_cmd[1] = (address >> 16) & 0xFF;
    read_cmd[2] = (address >> 8) & 0xFF;
    read_cmd[3] = address & 0xFF;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, read_cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/* USER CODE END 1 */
