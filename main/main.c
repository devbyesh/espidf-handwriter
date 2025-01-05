#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor.h"
#include "esp_timer.h"
#include "math.h"
#include "esp_check.h"

#define TAG "MAIN"

#define M1_STEP GPIO_NUM_18 // brown
#define M1_DIR GPIO_NUM_19  // red
#define M2_STEP GPIO_NUM_22 // orange
#define M2_DIR GPIO_NUM_23  // yellow

void app_main(void)
{
    // Set up GPIO for the motor
    gpio_config_t motor_gpio_config = {
        .pin_bit_mask = (1ULL << M1_STEP) | (1ULL << M1_DIR) | (1ULL << M2_STEP) | (1ULL << M2_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&motor_gpio_config);

    // set up motors on heap
    motor_t *m1 = malloc(sizeof(motor_t));
    motor_t *m2 = malloc(sizeof(motor_t));
    arm_linkage_2dof_t *arm = malloc(sizeof(arm_linkage_2dof_t));

    // initialize motors
    ESP_ERROR_CHECK(create_motor(m1, 1.8, SIXTEENTH_STEP, M1_STEP, M1_DIR));
    ESP_ERROR_CHECK(create_motor(m2, 1.8, SIXTEENTH_STEP, M2_STEP, M2_DIR));
    ESP_ERROR_CHECK(create_arm_linkage(arm, m1, m2, 150, 150));

    // test motors
    ESP_LOGI(TAG, "m1 to 90");
    ESP_ERROR_CHECK(move_motor_to(m1, .5, 90, false, NULL));
    vTaskDelay(1500/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "m1 to 180");
    ESP_ERROR_CHECK(move_motor_to(m1, 1, -90, true, NULL));
    ESP_ERROR_CHECK(move_motor_to(m1, 1, 0, true, NULL));

}
    
