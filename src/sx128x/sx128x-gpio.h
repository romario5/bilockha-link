#ifndef SX128X_GPIO_H_
#define SX128X_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"
#include <stdint.h>
#include "hal-gpio.h"


typedef void( GpioIrqHandler )( void );


void SX1281GpioWrite( uint16_t GPIO_Pin,  uint32_t value );

uint32_t SX1281GpioRead( uint16_t GPIO_Pin );


void SX1281GpioSetIrq( uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler );

void SX1281GpioLaunchIrqHandler( uint16_t GPIO_Pin );

#ifdef __cplusplus
}
#endif


#endif /* SX128X_GPIO_H_ */
