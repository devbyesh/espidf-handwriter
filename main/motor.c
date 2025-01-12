#include "freertos/FreeRTOS.h"
#include "motor.h"
#include "esp_check.h"

#define MIN_SECONDS_PER_REVOLUTION 6.0
#define MAX_DELAY_MICROSECONDS 10000
#define ACCELERATION_RATE .004f
#define DECELERATION_RATE -.004f
#define RAD_TO_DEG 57.2957795f
#define TAG "MOTOR"

float accel_profile(float last_velocity, float goal_velocity)
{
    return last_velocity + (goal_velocity - last_velocity > 0 ? 1 : -1) * ACCELERATION_RATE;
}

float decel_profile(int64_t steps_taken, float curr_velocity){
    return (curr_velocity > 0 ? 1 : -1) * (steps_taken * DECELERATION_RATE + fabsf(curr_velocity));
}

uint64_t decel_steps(float last_velocity, float goal_velocity){
    last_velocity = fabsf(last_velocity);
    goal_velocity = fabsf(goal_velocity);
    return (last_velocity - goal_velocity) / (-DECELERATION_RATE);
}

void change_timer_speed(esp_timer_handle_t handle, float speed, float min_delay){
    ESP_ERROR_CHECK(esp_timer_stop(handle));
    float delay = min_delay / fabsf(speed);
    delay = delay > MAX_DELAY_MICROSECONDS ? MAX_DELAY_MICROSECONDS : delay;
    ESP_ERROR_CHECK(esp_timer_start_periodic(handle, delay));
}

void move_motor_cb(void *cb_params)
{
    motor_timer_cb_params_t *params = cb_params;
    if(atomic_load(&params->can_move) && xSemaphoreTake(params->move_lock, 0) == pdTRUE){
        bool do_change_speed = false;

        //check if decel is needed
        if(llabs(params->goal_steps - params->curr_steps) <= decel_steps(params->goal_velocity, .1)){
            params->curr_velocity = decel_profile(llabs(params->curr_steps) - (llabs(params->goal_steps) - decel_steps(params->goal_velocity, .1)), params->goal_velocity);
            do_change_speed = true;
        }

        //check if accel is needed
        if(fabs(params->goal_velocity - params->curr_velocity) > .1 && !do_change_speed){ //only accel if not decel
            params->curr_velocity = accel_profile(params->curr_velocity, params->goal_velocity);
            do_change_speed = true;
        }

        if(do_change_speed){
            change_timer_speed(*params->handle, params->curr_velocity, params->min_delay);
        }

        //update gpio
        ESP_ERROR_CHECK(gpio_set_level(params->gpio_step, params->curr_steps % 2 == 0)); //2 ticks per step
        ESP_ERROR_CHECK(gpio_set_level(params->gpio_dir, params->curr_velocity > 0));

        //update position
        params->curr_steps += 1 * (params->curr_velocity > 0 ? 1 : -1);
        *params->curr_angle += params->step_angle/2 * (params->curr_velocity > 0 ? 1 : -1);
        

        //check if goal is reached
        if(llabs(params->goal_steps - params->curr_steps) == 0){
            //reset parameters
            params->curr_steps = 0;
            params->curr_velocity = 0;
            atomic_store(&params->can_move, false);
            change_timer_speed(*params->handle, 1, params->min_delay);

            //notify task if given
            if(params->calling_task_handle != NULL){
                TaskHandle_t calling_task_handle = params->calling_task_handle;
                params->calling_task_handle = NULL;
                xTaskNotifyGive(calling_task_handle);
            }
        }
        xSemaphoreGive(params->move_lock);
    }
}

esp_err_t create_motor(motor_t *m, float step_angle, step_mode_t step_mode, uint8_t gpio_step, uint8_t gpio_dir)
{
    if (m == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // initialize motor values
    m->step_angle = step_angle / (step_mode);
    m->min_delay = 1.0 / (360.0 / m->step_angle / (MIN_SECONDS_PER_REVOLUTION / 2)) * 1000000; // microseconds
    m->gpio_step = gpio_step;
    m->gpio_dir = gpio_dir;
    m->curr_angle = 0;

    ESP_LOGI(TAG, "step_angle: %f min_delay: %f", m->step_angle, m->min_delay);

    // initialize cb function values on heap
    motor_timer_cb_params_t *params = malloc(sizeof(motor_timer_cb_params_t));
    esp_timer_handle_t *handle = malloc(sizeof(esp_timer_handle_t));

    if (params == NULL || handle == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    params->gpio_step = gpio_step;
    params->gpio_dir = gpio_dir;
    params->curr_angle = &(m->curr_angle);
    params->step_angle = m->step_angle;
    params->min_delay = m->min_delay;
    params->handle = handle;
    params->goal_steps = 0;
    params->goal_velocity = 0;
    params->curr_velocity = 0;
    params->calling_task_handle = NULL;
    params->move_lock = xSemaphoreCreateMutex();
    params->curr_steps = 0;
    params->can_move = false;

    // create timer
    esp_timer_create_args_t config = {
        .arg = params,
        .callback = move_motor_cb,
        .name = "motor_timer",
    };

    ESP_RETURN_ON_ERROR(esp_timer_create(&config, handle), TAG, "Could not create timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(*handle, m->min_delay), TAG, "Could not start timer");

    m->timer_params = params;

    return ESP_OK;
}

esp_err_t move_motor_for(motor_t *m, float speed, uint64_t steps, bool do_block, TaskHandle_t handle)
{
    if (speed == 0)
    {
        return ESP_OK;
    }

    if (m == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    TaskHandle_t overwritten_handle = m->timer_params->calling_task_handle; 
    xSemaphoreTake(m->timer_params->move_lock, portMAX_DELAY);

    //set goal values
    int64_t required_steps = 2 * steps * (speed > 0 ? 1 : -1);
    if(llabs(required_steps) < 1){
        ESP_LOGW(TAG, "Motor already moving to goal");
        xSemaphoreGive(m->timer_params->move_lock);
        return ESP_OK;
    }
    
    m->timer_params->goal_steps = required_steps;
    m->timer_params->curr_steps = 0;
    m->timer_params->goal_velocity = speed;


    if (do_block) // if blocking, set the calling task handle so the cb function can notify it
    {
        m->timer_params->calling_task_handle = xTaskGetCurrentTaskHandle();
    }

    if(handle != NULL){ 
        if(overwritten_handle == NULL){
            m->timer_params->calling_task_handle = handle;
        } else{
            ESP_LOGW(TAG, "Cannot overwrite calling task handle if not blocking");
            xSemaphoreGive(m->timer_params->move_lock);
            return ESP_ERR_INVALID_ARG;
        }
    }

    atomic_store(&m->timer_params->can_move, true);
    xSemaphoreGive(m->timer_params->move_lock);

    if (do_block) //if blocking, wait for the cb function to notify the calling task and call overwritten handle if it exists
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(overwritten_handle != NULL){
            m->timer_params->calling_task_handle = overwritten_handle;
        }
    }

    return ESP_OK;
}

esp_err_t move_motor_to(motor_t *m, float speed, float target_angle, bool do_block, TaskHandle_t handle)
{
    xSemaphoreTake(m->timer_params->move_lock, portMAX_DELAY);
    atomic_store(&m->timer_params->can_move, false); //stop motor from changing angle between now and whenever move_motor_to is ready for it to start again
    float angle_to_mov = target_angle - m->curr_angle;

    bool spin_clockwise = angle_to_mov > 0;
    uint64_t steps = fabsf(angle_to_mov) / m->step_angle;
    xSemaphoreGive(m->timer_params->move_lock);

    float velocity = speed * (spin_clockwise ? 1 : -1);
    move_motor_for(m, velocity, steps, do_block, handle);
    return ESP_OK;
}

esp_err_t delete_motor(motor_t *m)
{
    ESP_RETURN_ON_ERROR(esp_timer_stop(*(m->timer_params->handle)), TAG, "Could not start timer");
    ESP_RETURN_ON_ERROR(esp_timer_delete(*(m->timer_params->handle)), TAG, "Could not delete timer");
    free(m->timer_params->handle);
    free(m->timer_params);

    return ESP_OK;
}

esp_err_t create_arm_linkage(arm_linkage_2dof_t *arm, motor_t *m1, motor_t *m2, float l1, float l2)
{
    if (arm == NULL || m1 == NULL || m2 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    arm->m1 = m1;
    arm->m2 = m2;
    arm->l1 = l1;
    arm->l2 = l2;

    return ESP_OK;
}

esp_err_t move_arm_to_angle(arm_linkage_2dof_t *arm, float a1, float a2){
    ESP_LOGI(TAG, "a1: %f, a2: %f", a1, a2);
    ESP_ERROR_CHECK(move_motor_to(arm->m1, 1, a1, false, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(move_motor_to(arm->m2, 1, a2, false, xTaskGetCurrentTaskHandle()));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return ESP_OK;
}

esp_err_t move_arm_to(arm_linkage_2dof_t *arm, float x, float y){
    //math & var names: https://www.desmos.com/calculator/tu4fdwxru9
    float d = sqrt(x*x + y*y);

    if (d > arm->l1 + arm->l2 && arm->l1 > 0 && arm->l2 > 0)
    {
        ESP_LOGE(TAG, "Target out of reach");
        return ESP_ERR_INVALID_ARG;
    }

    float phi1 = atan2(y, x);
    float phi2 = acos((arm->l1*arm->l1 + d*d - arm->l2*arm->l2) / (2 * arm->l1 * d));
    float theta1 = phi1 + phi2;
    float phi3 = acos((arm->l1*arm->l1 + arm->l2*arm->l2 - d*d) / (2 * arm->l1 * arm->l2));
    float theta2 = phi3 - M_PI;

    ESP_LOGI(TAG, "theta1: %f, theta2: %f", theta1, theta2);

    return move_arm_to_angle(arm, theta1 * RAD_TO_DEG, theta2 * RAD_TO_DEG);
}

void e_stop(motor_t *m){
    atomic_store(&m->timer_params->can_move, false);
    xSemaphoreTake(m->timer_params->move_lock, portMAX_DELAY);
    m->timer_params->curr_steps = 0;
    m->timer_params->curr_velocity = 0;
    if(m->timer_params->calling_task_handle != NULL){
        xTaskNotifyGive(m->timer_params->calling_task_handle);
        m->timer_params->calling_task_handle = NULL;
    }
    xSemaphoreGive(m->timer_params->move_lock);
}