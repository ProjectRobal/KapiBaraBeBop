#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_random.h>
#include <esp_littlefs.h>



#include "esp_dsp.h"
#include "esp_wifi.h"

#include <esp_log.h>

#include "WiFiManager.h"

#include "i2c.h"

#include "VL53L0X.h"
#include "Motor.h"


#define VLA_SDA GPIO_NUM_6
#define VLA_SCL GPIO_NUM_21

#define VLB_SDA GPIO_NUM_15
#define VLB_SCL GPIO_NUM_9


#define MAIN_TASK_STACK_SIZE 32768
#define MOTOR_TIMER LEDC_TIMER_1
#define MOTOR_FREQ 100


void main_task(void*arg);

void i2c_init();

esp_err_t init_littlefs(bool format_partion_on_failure,const char* partition_name);


TaskHandle_t xMainHandle = NULL;


void app_main(void)
{

    esp_log_level_set("MAIN",ESP_LOG_DEBUG);
    esp_log_level_set("TOF",ESP_LOG_DEBUG);

    init_littlefs(true,"littlefs");

    wifi_manager_init("BeBop","");

    xTaskCreatePinnedToCore(main_task,"main",MAIN_TASK_STACK_SIZE,NULL,configMAX_PRIORITIES,&xMainHandle,1-WIFIM_TASK_CORE_ID);

}


void main_task(void*arg)
{
    // init motors

    motor_timer_config(MOTOR_FREQ,MOTOR_TIMER);

    motor_t left=motor_new(LEDC_CHANNEL_0,LEDC_CHANNEL_1);

    // motor right goes full speed for some reason

    motor_init(&left,MOTOR_TIMER,GPIO_NUM_17,GPIO_NUM_16);

    motor_t right=motor_new(LEDC_CHANNEL_4,LEDC_CHANNEL_5);

    motor_init(&right,MOTOR_TIMER,GPIO_NUM_8,GPIO_NUM_18);

    motor_stop(&left);
    motor_stop(&right);
    
    // init distance sensors

    i2c_init();

    vl53l0x_t tofa=vl_new(I2C_NUM_0,GPIO_NUM_NC,GPIO_NUM_NC);

    vl_init(&tofa);
    
    vl_setTimingBudget(&tofa,35000);
    //vl_setSignalRateLimit(&tofa,0.25);
    //vl_setVcselPulsePeriodFinal(&tofa,8);
    //vl_setVcselPulsePeriodPre(&tofa,12);

    //vl_setContinousMode(&tofa);

    vl53l0x_t tofb=vl_new(I2C_NUM_1,GPIO_NUM_NC,GPIO_NUM_NC);

    vl_init(&tofb);

    vl_setTimingBudget(&tofb,35000);
    //vl_setSignalRateLimit(&tofb,0.25);
    //vl_setVcselPulsePeriodFinal(&tofb,8);
    //vl_setVcselPulsePeriodPre(&tofb,12);

    //vl_setContinousMode(&tofb);

    // network weights



    while(1)
    {

        vTaskDelay(50/portTICK_PERIOD_MS);

        bool connection_status = wifi_get_connection_status();

        wifi_ap_record_t ap_info;

        esp_wifi_sta_get_ap_info(&ap_info);

        ESP_LOGI("MAIN","SSID: %s",ap_info.ssid);

        // wait for connections
        // if( ap_info.ssid[0] == 0 )
        // {   
        //     ESP_LOGI("MAIN","Waiting for connection!");

        //     motor_stop(&left);
        //     motor_stop(&right);

        //     continue;
        // }

        // get Access Points informations

        

        // max distance of 100 mm 
        uint16_t distance_left = vl_read(&tofa);
        uint16_t distance_right = vl_read(&tofb);

        ESP_LOGI("MAIN","Left: %i",distance_left);
        ESP_LOGI("MAIN","Right: %i",distance_right);


        int32_t error = distance_left - distance_right;

        if( abs(error) < 100 )
        {
            motor_set_dir(&left,false);
            motor_set_speed(&left,2048);

            motor_set_dir(&right,false);
            motor_set_speed(&right,2048);
        }
        else if( error > 0 )
        {
            motor_set_dir(&left,false);
            motor_set_speed(&left,2048);

            motor_set_dir(&right,false);
            motor_set_speed(&right,0);
        }
        else
        {
            motor_set_dir(&left,false);
            motor_set_speed(&left,0);

            motor_set_dir(&right,false);
            motor_set_speed(&right,2048);
        }


        motor_update(&left);
        motor_update(&right);

        

        // motor_stop(&left);
        // motor_stop(&right);

    }

}

esp_err_t init_littlefs(bool format_partion_on_failure,const char* partition_name)
{
    esp_vfs_littlefs_conf_t conf={
        .base_path="/littlefs",
        .partition_label=partition_name,
        .format_if_mount_failed=format_partion_on_failure,
        .dont_mount=false,
    };

    return esp_vfs_littlefs_register(&conf);

}

void i2c_init()
{
    i2c_config_t conf1={0};

    conf1.mode=I2C_MODE_MASTER;
    conf1.sda_io_num=VLA_SDA;
    conf1.scl_io_num=VLA_SCL;
    conf1.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed=400000;
    //conf.clk_flags=0;

    esp_err_t err=ESP_OK;
    
    err=i2c_param_config(I2C_NUM_0,&conf1);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }

    conf1.mode=I2C_MODE_MASTER;
    conf1.sda_io_num=VLB_SDA;
    conf1.scl_io_num=VLB_SCL;
    conf1.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed=400000;
    //conf.clk_flags=0;

    err=ESP_OK;
    
    err=i2c_param_config(I2C_NUM_1,&conf1);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_NUM_1,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }
}

