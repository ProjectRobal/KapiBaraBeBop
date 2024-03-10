// library used for DRV8220 control

#pragma once

#include <driver/ledc.h>
#include <driver/gpio.h>

typedef struct motor
{
    ledc_channel_t chA;
    ledc_channel_t chB;
    bool dir;
    uint16_t speed;
} motor_t;

void motor_timer_config(uint32_t freq,ledc_timer_t timer);

motor_t motor_new(ledc_channel_t chA,ledc_channel_t chB);

void motor_init(motor_t* motor,ledc_timer_t timer,gpio_num_t pinA,gpio_num_t pinB);

void motor_set_dir(motor_t* motor,bool dir);

void motor_set_speed(motor_t* motor,uint16_t speed);

void motor_update(motor_t* motor);

void motor_stop(motor_t* motor);