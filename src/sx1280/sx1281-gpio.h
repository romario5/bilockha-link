/*
 * sx1281-hal-gpio.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Roman
 */

#ifndef SX1281_INC_SX1281_GPIO_H_
#define SX1281_INC_SX1281_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"
#include <stdint.h>
#include "gpio.h"


typedef void( GpioIrqHandler )( void );


void SX1281GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  uint32_t value );

uint32_t SX1281GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );


void SX1281GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler );

void SX1281GpioLaunchIrqHandler( uint16_t GPIO_Pin );

#ifdef __cplusplus
}
#endif


#endif /* SX1281_INC_SX1281_GPIO_H_ */
