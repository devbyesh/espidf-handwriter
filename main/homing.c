#include "motor.h"
#include "homing.h"

typedef struct cb_params{
    motor_t* motor;
    TaskHandle_t handle;
    limit_switch_t* sw;
} cb_params;

void home_motor_two_cb_1(void* args){
    cb_params* params = ((cb_params*) args);
    motor_t *motor = params->motor;
    TaskHandle_t handle = params->handle;
    params->sw->cb = NULL;

    e_stop(motor);
    motor->curr_angle = 0;
    move_motor_to(motor, 1, -360, false, NULL);
    xTaskNotifyGive(handle);
    free(params);
    ESP_LOGI("TAG", "Motor 2 PARAM FREED");
}

void home_motor_one_cb(void* args){
    cb_params* params = ((cb_params*) args);
    motor_t *motor = params->motor;
    TaskHandle_t handle = params->handle;
    params->sw->cb = NULL;

    e_stop(motor);
    motor->curr_angle = 0;
    xTaskNotifyGive(handle);
    free(params);
    ESP_LOGI("TAG", "Motor 1 homed");
}

void home_motor_two_cb_2(void* args){
    cb_params* params = ((cb_params*) args);
    motor_t *motor = params->motor;
    TaskHandle_t handle = params->handle;
    params->sw->cb = NULL;
    
    e_stop(motor);
    motor->curr_angle = motor->curr_angle/2;
    xTaskNotifyGive(handle);
    free(params);
    ESP_LOGI("TAG", "Motor 2 homed");
}


void home_motor_2s(motor_t* motor, limit_switch_t* left_limit_switch, limit_switch_t* right_limit_switch, TaskHandle_t handle){
    cb_params* paramsSW1 = malloc(sizeof(cb_params));
    cb_params* params = malloc(sizeof(cb_params));

    paramsSW1->motor = motor;
    paramsSW1->handle = handle;
    paramsSW1->sw = left_limit_switch;

    params->motor = motor;
    params->handle = handle;
    params->sw = right_limit_switch;
    
    limit_switch_change_cb(left_limit_switch, home_motor_two_cb_1, paramsSW1);
    limit_switch_change_cb(right_limit_switch, home_motor_two_cb_2, params);

    ESP_ERROR_CHECK(move_motor_to(motor, 1, 360, false, NULL));

}

void home_motor_1s(motor_t* motor, limit_switch_t* limit_switch, TaskHandle_t handle){
    cb_params* params = malloc(sizeof(cb_params));

    params->motor = motor;
    params->handle = handle;
    params->sw = limit_switch;

    limit_switch_change_cb(limit_switch, home_motor_one_cb, params);

    ESP_ERROR_CHECK(move_motor_to(motor, 1, -360, false, NULL));

}

void home_all_motors(motor_t* m1, motor_t* m2, limit_switch_t* left_limit_switch, limit_switch_t* right_limit_switch, limit_switch_t* m1_limit_switch){
    home_motor_2s(m2, left_limit_switch, right_limit_switch, xTaskGetCurrentTaskHandle());
    home_motor_1s(m1, m1_limit_switch, xTaskGetCurrentTaskHandle());

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI("TAG", "Motor 1 homed");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI("TAG", "Motor 2 homed1");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI("TAG", "Motor 2 homed2");
}