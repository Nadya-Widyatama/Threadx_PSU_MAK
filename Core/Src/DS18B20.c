/*
 * DS18B20.c
 *
 *  Created on: Dec 7, 2024
 *      Author: user
 */

#include "DS18B20.h"
#include "tx_api.h"
extern TIM_HandleTypeDef htim2;



void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Delay untuk mikrodetik
void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // Gunakan timer yang dikonfigurasi
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void thread_sleep_ms(uint32_t ms) {
    // Konversi milidetik ke tick ThreadX
    uint32_t tick = ms / (1000 / TX_TICK_RATE);  // Misalnya, jika 1 tick = 10 ms, maka TX_TICK_RATE = 100
    tx_thread_sleep(tick);
}


uint8_t DS18B20_Start(void) {
	uint8_t response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai output
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Pull pin low
	delay_us(480);  // Tahan selama 480us
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai input
	delay_us(80);  // Tunggu selama 80us
	if (!HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))response = 1;  // Jika pin low, sensor hadir
	else response = -1;
	delay_us(400);  // Tunggu selama 400us
	return response;
}

void DS18B20_Write(uint8_t data) {
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai output
	for (int i = 0; i < 8; i++) {
		if (data & (1 << i)) {  // Jika bit adalah 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai output
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Pull pin low
			delay_us(1);  // Tunggu selama 1us
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai input
			delay_us(60);  // Tunggu selama 60us
		} else {  // Jika bit adalah 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai output
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Pull pin low
			delay_us(60);  // Tunggu selama 60us
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai input
		}
	}
}

uint8_t DS18B20_Read(void) {
	uint8_t value = 0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai input
	for (int i = 0; i < 8; i++) {
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai output
		HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Pull pin low
		delay_us(2);  // Tunggu selama 2us
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Set pin sebagai input
		if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)) {  // Jika pin high
			value |= 1 << i;  // Simpan bit ke value
		}
		delay_us(60);  // Tunggu selama 60us
	}
	return value;
}

float DS18B20_GetTemp(void) {
	uint8_t temp_lsb, temp_msb;
	int16_t temp;

	DS18B20_Start();
	DS18B20_Write(0xCC);  // SKIP ROM
	DS18B20_Write(0x44);  // CONVERT T
	thread_sleep_ms(750000);

	DS18B20_Start();
	DS18B20_Write(0xCC);  // SKIP ROM
	DS18B20_Write(0xBE);  // READ SCRATCHPAD

	temp_lsb = DS18B20_Read();
	temp_msb = DS18B20_Read();
	temp = (temp_msb << 8) | temp_lsb;

	return (float)temp / 16.0;
}


