#include "gpio.h"
#include <sys/fcntl.h>




static void gpio_export(int pin)
{
    char buf[64];
    int fd = open(SYSFS_GPIO "/export", O_WRONLY);
    if (fd < 0) return;
    int len = snprintf(buf, sizeof(buf), "%d", pin);
    write(fd, buf, len);
    close(fd);
}


void gpio_set_dir(int pin, gpio_dir_t dir)
{
    char path[128];
    snprintf(path, sizeof(path), SYSFS_GPIO "/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return;
    write(fd, dir, strlen(dir));
    close(fd);
}


gpio_fd_t gpio_open_value(int pin, gpio_mode_t mode)
{
    char path[128];
    snprintf(path, sizeof(path), SYSFS_GPIO "/gpio%d/value", pin);
    return open(path, mode == GPIO_MODE_W ? O_WRONLY : O_RDONLY);
}

gpio_fd_t GPIO_Open(int pin, gpio_dir_t dir)
{
	gpio_export(pin);
	gpio_set_dir(pin, dir);
	return gpio_open_value(pin, dir == GPIO_DIR_IN ? GPIO_MODE_R : GPIO_MODE_W);
}


void HAL_GPIO_WritePin(gpio_fd_t fd, uint8_t val)
{
    lseek(fd, 0, SEEK_SET);
    write(fd, val ? "1" : "0", 1);
}


uint8_t HAL_GPIO_ReadPin(gpio_fd_t fd)
{
    char c;
    lseek(fd, 0, SEEK_SET);
    read(fd, &c, 1);
    return c == '1' ? GPIO_VAL_HIGH : GPIO_VAL_LOW;
}