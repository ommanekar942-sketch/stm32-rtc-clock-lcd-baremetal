/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jul 16, 2025
 *      Author: DELL
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

uint32_t RCC_GetPLLOutputClock(void);

// this return the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

// this return the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
