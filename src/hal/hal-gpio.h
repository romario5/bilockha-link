#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef uint8_t  gpio_status_t;
typedef int      gpio_fd_t;
typedef uint8_t  gpio_mode_t;
typedef char*    gpio_dir_t;


extern const gpio_status_t GPIO_OK;
extern const gpio_status_t GPIO_ER;

extern const uint8_t GPIO_VAL_HIGH;
extern const uint8_t GPIO_VAL_LOW;

extern const gpio_mode_t GPIO_MODE_R;
extern const gpio_mode_t GPIO_MODE_W;

extern const gpio_dir_t GPIO_DIR_IN;
extern const gpio_dir_t GPIO_DIR_OUT;

void HAL_GPIO_Open(int pin, gpio_dir_t dir);
void HAL_GPIO_WritePin(int pin, uint8_t value);
uint8_t HAL_GPIO_ReadPin(int pin);


#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
