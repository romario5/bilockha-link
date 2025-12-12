#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C" {
#endif


typedef uint8_t  gpio_status_t;
typedef int      gpio_fd_t;
typedef uint8_t  gpio_mode_t;
typedef char*    gpio_dir_t;


const gpio_status_t GPIO_OK = 1;
const gpio_status_t GPIO_ER = 0;

const uint8_t GPIO_VAL_HIGH = 1;
const uint8_t GPIO_VAL_LOW  = 0;

const gpio_mode_t GPIO_MODE_R = 0;
const gpio_mode_t GPIO_MODE_W = 1;

const gpio_dir_t GPIO_DIR_IN  = "in";
const gpio_dir_t GPIO_DIR_OUT = "out";

gpio_fd_t HAL_GPIO_Open(int pin, gpio_dir_t dir, gpio_mode_t mode);
void HAL_GPIO_WritePin(gpio_fd_t fd, uint8_t value);
uint8_t HAL_GPIO_ReadPin(gpio_fd_t fd);


#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */
