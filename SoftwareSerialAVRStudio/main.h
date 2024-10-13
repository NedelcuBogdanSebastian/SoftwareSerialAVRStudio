/*
 * main.h
 *
 * Created: 8/15/2024 6:22:45 PM
 *  Author: Bogdan
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "port.h"

//extern u16 usRegInputBuf[100+1];
//extern u16 usRegHoldingBuf[100+1];
//extern u8 usRegCoilBuf[64/8+1];    // We have 64 coils

uint16_t usRegHoldingBuf[100]; // 0..99 holding registers
uint8_t  usRegCoilBuf[72/8];   // 0..71 coils


#endif /* MAIN_H_ */