#include "limit_switch.h"
#include "freertos/timers.h"
#include "esp_log.h"

void limit_switch_isr(void *arg){
    limit_switch_t* limit_switch = (limit_switch_t*)arg;
    xTimerResetFromISR(limit_switch->timer, NULL);
}

void limit_switch_debounce(TimerHandle_t timer){
    limit_switch_t* limit_switch = (limit_switch_t*)pvTimerGetTimerID(timer);
    limit_switch->triggered = gpio_get_level(limit_switch->gpio) == 0;
    if(limit_switch->cb != NULL){
        limit_switch->cb(limit_switch->args);
    }
}


void limit_switch_init(limit_switch_t* limit_switch, gpio_num_t gpio, limit_switch_cb_t cb){
    limit_switch->gpio = gpio;
    gpio_config_t limit_switch_gpio_config = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&limit_switch_gpio_config);
    limit_switch->triggered = false;
    limit_switch->timer = xTimerCreate("limit_switch_timer", pdMS_TO_TICKS(10), false, limit_switch, limit_switch_debounce);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio, limit_switch_isr, (void*)limit_switch);
}

void limit_switch_change_cb(limit_switch_t* limit_switch, limit_switch_cb_t cb, void *arg){
    limit_switch->cb = cb;
    limit_switch->args = arg;
}
