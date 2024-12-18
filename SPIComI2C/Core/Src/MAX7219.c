/*
 * MAX7219.c
 *
 *  Created on: 16 Jan 2023
 *      Author: EDISON NGUNJIRI
 */
#include "stm32h7xx_hal.h"
#include "MAX7219.h"
#define CHAR_MAX 38
uint8_t decode[2];
extern SPI_HandleTypeDef hspi1;
uint8_t data[2];
uint8_t shutdown[2];

uint8_t i=0;
uint8_t ON=0xFF;
uint8_t OFF=0x00;

const uint8_t InitCommands[10][2]={
		{0x09,0x00},
		{0x0A,0x00},
		{0x0B,0x07},
		{0x0C,0x01},
		{0x0F,0x00}
};
uint8_t LETTERS[CHAR_MAX][8]={
		{0x00,0x08,0x14,0x14,0x14,0x14,0x08,0x00},/*0*/
		{0x00,0x1C,0x08,0x08,0x08,0x0C,0x08,0x00},/*1*/
		{0x00,0x3C,0x04,0x18,0x20,0x20,0x1C,0x00},/*2*/
		{0x00,0x1C,0x20,0x20,0x1C,0x20,0x1C,0x00},/*3*/
		{0x00,0x10,0x10,0x3C,0x14,0x14,0x04,0x00},/*4*/
		{0x00,0x1C,0x20,0x24,0x1C,0x04,0x3C,0x00},/*5*/
		{0x00,0x1C,0x24,0x24,0x1C,0x04,0x38,0x00},/*6*/
		{0x00,0x10,0x10,0x38,0x10,0x20,0x3C,0x00},/*7*/
		{0x00,0x18,0x24,0x24,0x18,0x24,0x18,0x00},/*8*/
		{0x00,0x04,0x08,0x18,0x24,0x24,0x18,0x00},/*9*/
		{0x00,0x24,0x24,0x3C,0x24,0x24,0x18,0x00},/*A*/
		{0x00,0x1C,0x24,0x24,0x1C,0x24,0x1C,0x00},/*B*/
		{0x00,0x38,0x04,0x04,0x04,0x04,0x38,0x00},/*C*/
		{0x00,0x1C,0x24,0x24,0x24,0x24,0x1C,0x00},/*D*/
		{0x00,0x3C,0x04,0x3C,0x04,0x3C,0x00,0x00},/*E*/
		{0x00,0x04,0x04,0x1C,0x04,0x04,0x3C,0x00},/*F*/
		{0x00,0x38,0x24,0x34,0x04,0x24,0x18,0x00},/*G*/
		{0x00,0x24,0x24,0x3C,0x24,0x24,0x00,0x00},/*H*/
		{0x00,0x1C,0x08,0x08,0x08,0x1C,0x00,0x00},/*I*/
		{0x00,0x1C,0x10,0x10,0x10,0x38,0x00,0x00},/*J*/
		{0x00,0x24,0x14,0x0C,0x14,0x24,0x04,0x00},/*K*/
		{0x00,0x3C,0x04,0x04,0x04,0x04,0x04,0x00},/*L*/
		{0x00,0x22,0x22,0x2A,0x36,0x22,0x00,0x00},/*M*/
		{0x00,0x22,0x32,0x2A,0x26,0x22,0x22,0x00},/*N*/
		{0x00,0x18,0x24,0x24,0x24,0x24,0x18,0x00},/*O*/
		{0x00,0x04,0x04,0x1C,0x24,0x24,0x1C,0x00},/*P*/
		{0x40,0x38,0x34,0x24,0x24,0x24,0x18,0x00},/*Q*/
		{0x00,0x24,0x14,0x1C,0x24,0x24,0x1C,0x00},/*R*/
		{0x18,0x24,0x20,0x18,0x04,0x24,0x18,0x00},/*S*/
		{0x00,0x08,0x08,0x08,0x08,0x08,0x3E,0x00},/*T*/
		{0x00,0x18,0x24,0x24,0x24,0x24,0x00,0x00},/*U*/
		{0x00,0x08,0x14,0x22,0x22,0x22,0x22,0x00},/*V*/
		{0x00,0x44,0x6C,0x54,0x44,0x44,0x00,0x00},/*W*/
		{0x00,0x28,0x28,0x10,0x28,0x28,0X00,0x00},/*X*/
		{0x10,0x10,0x10,0x18,0x24,0x24,0x24,0x00},/*Y*/
		{0x00,0x7C,0x08,0x10,0x20,0x7C,0x00,0x00},/*Z*/
		{0x00,0x18,0x3C,0x7E,0x7F,0x77,0x23,0x00},/*HEART ON*/
		{0xFF,0xE7,0xC3,0xC1,0x81,0x81,0xCB,0xFF},/*HEART OFF*/
};


void MAX72_Init_F(void)
{
	uint8_t i=0;
	for(i=0;i<5;i++)
	{
		shutdown[0]=InitCommands[i][0];
		shutdown[1]=InitCommands[i][1];
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1,shutdown,2, 1000);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(10);
	}
}

void Print_Alphabet(void)
{
	for(int i=0;i<=CHAR_MAX;i++)
	{
		for(int k=1;k<=8;k++)
		{
			shutdown[0]=k;
			shutdown[1]=LETTERS[i-1][k-1];
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,shutdown,2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, GPIO_PIN_SET);
		}
		HAL_Delay(1000);
	}
}
