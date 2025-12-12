#include "sx128x-gpio.h"

static GpioIrqHandler *GpioIrq[146] = { NULL };


/*!
 * @brief Get the position of the bit set in the GPIO_Pin
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval the position of the bit
 */
uint8_t GpioGetBitPos( uint16_t GPIO_Pin )
{
    return (uint8_t) GPIO_Pin;
}


/*!
 * @brief Records the interrupt handler for the GPIO  object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] prio       NVIC priority (0 is highest)
 * @param [IN] irqHandler  points to the  function to execute
 * @retval none
 */
void SX1281GpioSetIrq( uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
{
    uint32_t BitPos = GpioGetBitPos( GPIO_Pin );
    if ( irqHandler != NULL )
    {
        GpioIrq[BitPos] = irqHandler;
    }
}

/*!
 * @brief Execute the interrupt from the object
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
void SX1281GpioLaunchIrqHandler( uint16_t GPIO_Pin )
{
    uint32_t BitPos = GpioGetBitPos( GPIO_Pin );

    if ( GpioIrq[BitPos]  != NULL )
    {
        GpioIrq[BitPos]( );
    }
}




/*!
 * @brief Writes the given value to the GPIO output
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] value New GPIO output value
 * @retval none
 */
void SX1281GpioWrite( uint16_t GPIO_Pin, uint32_t value )
{
    HAL_GPIO_WritePin( GPIO_Pin , value );
}

/*!
 * @brief Reads the current GPIO input value
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval value   Current GPIO input value
 */
uint32_t SX1281GpioRead( uint16_t GPIO_Pin )
{
    return HAL_GPIO_ReadPin( GPIO_Pin );
}

