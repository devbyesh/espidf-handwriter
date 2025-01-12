#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "freertos/timers.h"

typedef void (*limit_switch_cb_t)(void* args);

typedef struct{
    gpio_num_t gpio;
    bool triggered;
    TimerHandle_t timer;
    limit_switch_cb_t cb;
    void* args;
} limit_switch_t;

void limit_switch_init(limit_switch_t* limit_switch, gpio_num_t gpio, limit_switch_cb_t cb);
void limit_switch_change_cb(limit_switch_t* limit_switch, limit_switch_cb_t cb, void *arg);
#endif