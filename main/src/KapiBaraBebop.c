#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_random.h>
#include <esp_littlefs.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#include "esp_dsp.h"

#include <esp_log.h>

#include "WiFiManager.h"

#include "i2c.h"

#include "VL53L0X.h"
#include "Motor.h"

#include "network.h"
#include "network_serializer.h"

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

void micro_ros_task(void * arg);

TaskHandle_t xMainHandle = NULL;
TaskHandle_t xRosHandle = NULL;


void app_main(void)
{

    esp_log_level_set("MAIN",ESP_LOG_DEBUG);
    esp_log_level_set("TOF",ESP_LOG_DEBUG);

    init_littlefs(true,"littlefs");

    wifi_manager_init("BeBop","");

    xTaskCreatePinnedToCore(main_task,"main",MAIN_TASK_STACK_SIZE,NULL,configMAX_PRIORITIES,&xMainHandle,1-WIFIM_TASK_CORE_ID);
    xTaskCreatePinnedToCore(micro_ros_task,"ros",MAIN_TASK_STACK_SIZE,NULL,configMAX_PRIORITIES,&xRosHandle,1-WIFIM_TASK_CORE_ID);

}


float clip(float x)
{
    if(x>1.f)
    {
        return 1.f;
    }

    return x;
}

volatile static float mutation_rate=0.1;

void mutation_layer(layer_t* layer)
{
    for(size_t i=0;i<layer->size;++i)
    {

        for(size_t n=0;n<layer->neurons[i].size;++n)
        {
            if( mutation_rate > (((float)esp_random())/RAND_MAX) )
            {
                layer->neurons[i].input_weights[n]+=gauss(0.f,0.01f);
            }
        }

        if( mutation_rate > (((float)esp_random())/RAND_MAX) )
        {
                layer->neurons[i].bias+=gauss(0.f,0.01f);
        }

    }
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
    FILE* file;

    layer_t input_layer;

    file=fopen("/littlefs/layer1.bin","r");
    if((file != NULL)&&load_layer_from_file(file,&input_layer))
    {
        ESP_LOGI("MAIN","Loaded layer 1 from file!");
        input_layer.filter=&relu;
        fclose(file);
    }
    else
    {
        input_layer=new_layer(64,2,&relu);
    }
    
    layer_t output_layer;

    file=fopen("/littlefs/layer2.bin","r");
    if((file != NULL )&&load_layer_from_file(file,&output_layer))
    {
        ESP_LOGI("MAIN","Loaded layer 2 from file!");
        output_layer.filter=&relu;
        fclose(file);
    }
    else
    {
        output_layer=new_layer(4,64,&relu);
    }
    
    input_layer.next=&output_layer;

    float inputs[2]={0.f};

    float output[4]={0};

    uint16_t save_ticks=0;

    while(1)
    {
        // max distance of 100 mm 
        inputs[0]=clip(vl_read(&tofa)/100.f);
        inputs[1]=clip(vl_read(&tofb)/100.f);

        //ESP_LOGI("MAIN","Left: %f",inputs[0]);
        //ESP_LOGI("MAIN","Right: %f",inputs[1]);

        // decision

        fire_network(&input_layer,inputs,output);

        /*for(size_t i=0;i<4;i++)
        {
            ESP_LOGI("MAIN","Output %d: %f",i,output[i]);
        }*/

        // update weights in layer
        mutation_layer(&input_layer);
        mutation_layer(&output_layer);

        if(save_ticks==2400)
        {
            save_ticks=0;

            FILE* file=fopen("/littlefs/layer1.bin","w");

            save_layer_to_file(file,&input_layer);

            fclose(file);

            file=fopen("/littlefs/layer2.bin","w");

            save_layer_to_file(file,&output_layer);

            fclose(file);

            ESP_LOGI("MAIN","Saved layers to file!");

        }

        // higher error means higher mutation rate
        float error=((1.f-inputs[0])+(1.f-inputs[1]))/2.f;

        mutation_rate=0.55*error+0.45*mutation_rate;
        
        ESP_LOGI("MAIN","Mutation rate: %f",mutation_rate);

        //step
        
        motor_set_dir(&left,output[0]>=output[1]);
        motor_set_speed(&left,2048);

        motor_set_dir(&right,output[2]>=output[3]);
        motor_set_speed(&right,2048);

        motor_update(&left);
        motor_update(&right);

        vTaskDelay(25/portTICK_PERIOD_MS);

        motor_stop(&left);
        motor_stop(&right);

        save_ticks++;
    }

    free_network(&input_layer);
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


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		ESP_LOGI("MAIN","Publishing: %d\n", (int) msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(10/portTICK_PERIOD_MS);

	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}