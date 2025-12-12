#include "hal-gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fcntl.h>

const gpio_status_t GPIO_OK = 1;
const gpio_status_t GPIO_ER = 0;

const uint8_t GPIO_VAL_HIGH = 1;
const uint8_t GPIO_VAL_LOW  = 0;

const gpio_mode_t GPIO_MODE_R = 0;
const gpio_mode_t GPIO_MODE_W = 1;

const gpio_dir_t GPIO_DIR_IN  = "in";
const gpio_dir_t GPIO_DIR_OUT = "out";

const char value_path_tpl[] = "/sys/class/gpio/gpio%d/value";
char value_path[128];
char cat_command[128];

FILE* files[200] = {0};

static void gpio_export(int pin)
{
    FILE* file = fopen("/sys/class/gpio/export", "w");
    if (file == NULL) {
        perror("Failed to open GPIO export file");
        return;
    }
    fprintf(file, "%d", pin);
    fclose(file);
}


void gpio_set_dir(int pin, gpio_dir_t dir)
{
    char direction_path[128];
    snprintf(direction_path, sizeof(direction_path), "/sys/class/gpio/gpio%d/direction", pin);
    FILE* file = fopen(direction_path, "w");
    if (file == NULL) {
        perror("Failed to open GPIO direction file");
        return;
    }
    fprintf(file, dir == GPIO_DIR_IN ? "in" : "out");
    fclose(file);
}


void gpio_open_value(int pin, gpio_dir_t dir)
{
    memset(value_path, 0, sizeof(value_path));
    snprintf(value_path, sizeof(value_path), value_path_tpl, pin);
    FILE* file = fopen(value_path, dir == GPIO_DIR_IN ? "r" : "w");
    if (file == NULL) {
        perror("Failed to open GPIO value file");
        return;
    }
    files[pin] = file;
}

void HAL_GPIO_Open(int pin, gpio_dir_t dir)
{
    gpio_export(pin);
    gpio_set_dir(pin, dir);
    gpio_open_value(pin, dir);
}


void HAL_GPIO_WritePin(int pin, uint8_t val)
{
    if (files[pin] == NULL) return;
    fprintf(files[pin], val == 1 ? "1" : "0");
    fflush(files[pin]);
}


uint8_t HAL_GPIO_ReadPin(int pin)
{
    if (files[pin] == NULL) return 0;
    
    memset(value_path, 0, sizeof(value_path));
    snprintf(value_path, sizeof(value_path), value_path_tpl, pin);
    
    int value;
    fscanf(files[pin], "%d", &value);
    return (uint8_t)value;
}

