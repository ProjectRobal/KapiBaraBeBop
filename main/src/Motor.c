#include "Motor.h"

void motor_timer_config(uint32_t freq,ledc_timer_t timer)
{
    ledc_timer_config_t timer_cfg;
    timer_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution=LEDC_TIMER_12_BIT;
    timer_cfg.freq_hz=freq;
    timer_cfg.timer_num=timer;
    timer_cfg.clk_cfg=LEDC_USE_RC_FAST_CLK;
    timer_cfg.deconfigure=false;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
}


motor_t motor_new(ledc_channel_t chA,ledc_channel_t chB)
{
    motor_t m={
        .chA=chA,
        .chB=chB,
        .speed=0,
        .dir=false
    };
    return m;
}

void motor_init(motor_t* motor,ledc_timer_t timer,gpio_num_t pinA,gpio_num_t pinB)
{

    gpio_set_direction(pinA,GPIO_MODE_OUTPUT);
    gpio_set_direction(pinB,GPIO_MODE_OUTPUT);

    ledc_channel_config_t channel_cfg;

    channel_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    channel_cfg.intr_type=LEDC_INTR_DISABLE;
    channel_cfg.duty=0;
    channel_cfg.timer_sel=timer;
    channel_cfg.flags.output_invert=true;
    channel_cfg.hpoint=(1<<12)-1;

    channel_cfg.gpio_num=pinA;
    channel_cfg.channel=motor->chA;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

    channel_cfg.gpio_num=pinB;
    channel_cfg.channel=motor->chB;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

}

void motor_set_dir(motor_t* motor,bool dir)
{
    motor->dir=dir;
}

void motor_set_speed(motor_t* motor,uint16_t speed)
{
    if( speed>=4095 )
    {
        speed=4095;
    }

    motor->speed=speed;
}

void motor_update(motor_t* motor)
{
    if(motor->dir)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,motor->chA,motor->speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,motor->chB,0);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,motor->chA,0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,motor->chB,motor->speed);
    }
    

    ledc_update_duty(LEDC_LOW_SPEED_MODE,motor->chA);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,motor->chB);
}

void motor_stop(motor_t* motor)
{
    motor->speed=0;
    motor_update(motor);
}