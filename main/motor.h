#ifndef MOTOR
#define MOTOR
#include "math.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"


typedef struct {
    //motion
    int64_t goal_steps;
    int64_t curr_steps;
    float goal_velocity;
    float curr_velocity;
    float min_delay;

    //gpio config
    uint8_t gpio_step;
    uint8_t gpio_dir;

    //angle tracking
    float* curr_angle;
    float step_angle;

    //syncronization
    SemaphoreHandle_t move_lock;
    esp_timer_handle_t* handle;
    TaskHandle_t calling_task_handle;
    atomic_bool can_move;

    bool debug;
    
} motor_timer_cb_params_t;

typedef struct motor_s{
    float step_angle;
    float min_delay;
    motor_timer_cb_params_t* timer_params;
    uint8_t gpio_step;
    uint8_t gpio_dir;
    float curr_angle;
} motor_t;

typedef struct {
    motor_t* m1;
    motor_t* m2;
    float l1;
    float l2;
} arm_linkage_2dof_t;

typedef enum{
    WHOLE_STEP = 1,
    HALF_STEP = 2,
    QUARTER_STEP = 4,
    EIGHTH_STEP = 8,
    SIXTEENTH_STEP = 16
} step_mode_t;

esp_err_t create_motor(motor_t* m, float step_angle, step_mode_t step_mode, uint8_t gpio_step, uint8_t gpio_dir);
esp_err_t move_motor_for(motor_t *m, float speed, uint64_t steps, bool do_block, TaskHandle_t handle);
esp_err_t move_motor_to(motor_t *m, float speed, float target_angle, bool do_block, TaskHandle_t handle);
esp_err_t create_arm_linkage(arm_linkage_2dof_t *arm, motor_t *m1, motor_t *m2, float l1, float l2);
esp_err_t move_arm_to(arm_linkage_2dof_t *arm, float x, float y);
esp_err_t move_arm_to_angle(arm_linkage_2dof_t *arm, float a1, float a2);
void e_stop(motor_t *m);

#endif