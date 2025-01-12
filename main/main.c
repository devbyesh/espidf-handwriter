#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor.h"
#include "esp_timer.h"
#include "math.h"
#include "esp_check.h"
#include "limit_switch.h"
#include "stdatomic.h"
#include "homing.h"

#define TAG "MAIN"

#define M1_STEP GPIO_NUM_18 // brown
#define M1_DIR GPIO_NUM_19  // red
#define M2_STEP GPIO_NUM_22 // orange
#define M2_DIR GPIO_NUM_23  // yellow

#define LEFT_LIMIT_SWITCH GPIO_NUM_16
#define RIGHT_LIMIT_SWITCH GPIO_NUM_17
#define M1_LIMIT_SWITCH GPIO_NUM_21

void app_main(void)
{
    // set up gpio for the motor
    gpio_config_t motor_gpio_config = {
        .pin_bit_mask = (1ULL << M1_STEP) | (1ULL << M1_DIR) | (1ULL << M2_STEP) | (1ULL << M2_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&motor_gpio_config);

    // set up motors on heap
    motor_t *m1 = malloc(sizeof(motor_t));
    motor_t *m2 = malloc(sizeof(motor_t));
    arm_linkage_2dof_t *arm = malloc(sizeof(arm_linkage_2dof_t));

    // initialize motors
    ESP_ERROR_CHECK(create_motor(m1, 1.8, SIXTEENTH_STEP, M1_STEP, M1_DIR));
    ESP_ERROR_CHECK(create_motor(m2, 1.8, SIXTEENTH_STEP, M2_STEP, M2_DIR));
    ESP_ERROR_CHECK(create_arm_linkage(arm, m1, m2, 150, 150));

    // set up limit switches
    limit_switch_t *left_limit_switch = malloc(sizeof(limit_switch_t));
    limit_switch_t *right_limit_switch = malloc(sizeof(limit_switch_t));
    limit_switch_t *m1_limit_switch = malloc(sizeof(limit_switch_t));
    limit_switch_init(left_limit_switch, LEFT_LIMIT_SWITCH, NULL);
    limit_switch_init(right_limit_switch, RIGHT_LIMIT_SWITCH, NULL);
    limit_switch_init(m1_limit_switch, M1_LIMIT_SWITCH, NULL);

    // home motors
    home_all_motors(m1, m2, left_limit_switch, right_limit_switch, m1_limit_switch);

    // move arm
    move_motor_to(m1, 1, 90, false, NULL);

    move_arm_to(arm, 150, 150);
    move_arm_to(arm, 0, 150);
    move_arm_to(arm, 150, 0);
}


    
