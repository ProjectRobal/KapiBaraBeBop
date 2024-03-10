#include <stdio.h>
#include <string.h>

#include <esp_wifi.h>
#include <esp_netif_net_stack.h>
#include <esp_netif.h>
#include <nvs_flash.h>

#include <esp_log.h>

#include <esp_littlefs.h>

#include "WiFiManager.h"
#include "HttpService.h"

#include <freertos/event_groups.h>

static EventGroupHandle_t xWiFiStatus;

static TaskHandle_t xWifiHandle = NULL;

static uint8_t ConnectionAttempts = 0;

esp_err_t wifi_manager_init_littlefs(bool format_partion_on_failure,const char* partition_name)
{
    esp_vfs_littlefs_conf_t conf={
        .base_path="/littlefs",
        .partition_label=partition_name,
        .format_if_mount_failed=format_partion_on_failure,
        .dont_mount=false,
    };

    return esp_vfs_littlefs_register(&conf);

}

bool wifi_manager_read_network_from_flash(char *ssid,char* password)
{
    FILE *file=fopen(WIFI_MANAGER_NETWORK_FILENAME,"r");

    if(!file)
    {
        return false;
    }

    fscanf(file,"%s\n%s",ssid,password);

    fclose(file);

    return true;
}

void wifi_manager_save_network_to_flash(const char *ssid,const char* password)
{

    FILE *file=fopen(WIFI_MANAGER_NETWORK_FILENAME,"w");

    if(!file)
    {
        ESP_LOGD("WifiManager","Cannot save network data into flash");
        return;
    }

    fprintf(file,"%.32s\n%.64s\n",ssid,password);

    fclose(file);
}


void wifi_manager_wifi_event_hadler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
   
   if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
        break;
        // wifi scan done
        case WIFI_EVENT_SCAN_DONE:

        break;

        // someone connected to stationary access point
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI("WiFiManager","Device connected, starting config server");
            start_http();

        break;

        // someone disconnected to stationary access point
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI("WiFiManager","Device disconnected, stoping config server");
            stop_http();

        break;

        // device connected to wi-fi
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("WiFiManager","ESP connected to network");
            xEventGroupClearBits(xWiFiStatus,EVENT_BITS_WIFI_DISCONNECTED);
        break;

        // device disconnected to wi-fi
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI("WiFiManager","ESP disconnected to network, preparing for scanning");
            xEventGroupSetBits(xWiFiStatus,EVENT_BITS_WIFI_DISCONNECTED);
        break;
        }

    }
    else if(event_base == IP_EVENT)
    {

        switch(event_id)
    {
        // wifi scan done, wich means we have succesfully connected to network!
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI("WiFiManager","ESP got IP from Access Point!");
        break;
    }

    }

}

void wifi_manager_init_ap(const char* ssid,const char* psk)
{
    esp_netif_create_default_wifi_ap(); 

    wifi_config_t wifi_config;

    if(strlen(ssid) == 0)
    {
        strncpy((char*)wifi_config.ap.ssid,DEFAULT_WIFI_MANAGER_AP_SSID,32);
        wifi_config.ap.ssid_len=strlen(DEFAULT_WIFI_MANAGER_AP_SSID);
    }
    else
    {
        strncpy((char*)wifi_config.ap.ssid,ssid,32);
        wifi_config.ap.ssid_len=strlen(ssid);
    }

    if(strlen(psk)!=0)
    {
        strncpy((char*)wifi_config.ap.password,psk,32);
        wifi_config.ap.authmode=WIFI_AUTH_WPA2_PSK;
    }
    else
    {
        wifi_config.ap.authmode=WIFI_AUTH_OPEN;
    }

    wifi_config.ap.max_connection=1;
    wifi_config.ap.channel=1;
    wifi_config.ap.pmf_cfg.required=false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&wifi_config));

}

void wifi_manager_turn_on_ap(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
}

void wifi_manager_stop_ap(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

void wifi_manager_connect_to_ssid(const char* ssid,const char* pass)
{
    ConnectionAttempts=0;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_disconnect();

    // function that will set ssid and password for station
    wifi_config_t wifi_sta_config={
        .sta={
            .scan_method=WIFI_FAST_SCAN,
            .failure_retry_cnt=5,
            .sort_method=WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi=-127,
            .threshold.authmode=WIFI_AUTH_OPEN,
        }
    };

    strncpy((char*)wifi_sta_config.sta.ssid,ssid,32);
    strncpy((char*)wifi_sta_config.sta.password,pass,64);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&wifi_sta_config));
}

void wifi_manager_new_ssid_set()
{
    xEventGroupSetBits(xWiFiStatus,EVENT_BITS_NEW_SSID);
}

void wifi_manager_loop(void* arg)
{
    EventBits_t uxBits;

    while(1)
    {

    uxBits=xEventGroupGetBits(xWiFiStatus);

    if(((uxBits & EVENT_BITS_WIFI_DISCONNECTED)||(uxBits & EVENT_BITS_NEW_SSID )) && (ConnectionAttempts < WIFI_CONNECTION_ATTEMPTS))
    {

    esp_wifi_connect();

    ConnectionAttempts++;

    xEventGroupClearBits(xWiFiStatus,EVENT_BITS_NEW_SSID);

    if(ConnectionAttempts>=WIFI_CONNECTION_ATTEMPTS)
    {
        esp_wifi_set_mode(WIFI_MODE_AP);
    }

    }

    // call for each 5 seconds
    vTaskDelay(5000/portTICK_PERIOD_MS);

    }
}

void wifi_manager_init_wifi(void)
{

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  
}

void wifi_manager_init(const char* AP_SSID,const char* AP_PSK)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_manager_init_wifi();

    xWiFiStatus=xEventGroupCreate();

    xTaskCreatePinnedToCore(wifi_manager_loop,"WiFiManager",WIFI_TASK_STACK_SIZE,NULL,tskIDLE_PRIORITY,&xWifiHandle,WIFIM_TASK_CORE_ID);

    esp_netif_create_default_wifi_sta(); 

    wifi_manager_init_ap(AP_SSID,AP_PSK);

    /*esp_err_t ret= wifi_manager_init_littlefs(format_partition_on_failure,partiton_name);

    if ( ret != ESP_OK)
        {
                if (ret == ESP_FAIL)
                {
                        ESP_LOGE("WifiManager", "Failed to mount or format filesystem");
                }
                else if (ret == ESP_ERR_NOT_FOUND)
                {
                        ESP_LOGE("WifiManager", "Failed to find LittleFS partition");
                }
                else
                {
                        ESP_LOGE("WifiManager", "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
                }
        }
    */

    // check for network ssid and passwork in littlefs file

    /*char ssid[33];
    char password[65];

    if(wifi_manager_read_network_from_flash(ssid,password))
    {
        ESP_LOGI("WiFiManager","Found network with ssid: %.32s",ssid);
        ESP_LOGD("WiFiManager","Network password: %.64s",password);
        wifi_manager_connect_to_ssid(ssid,password);

    }
    else
    {
        ESP_LOGE("WiFiManager","Found no saved networks!");
    }*/


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
    ESP_EVENT_ANY_ID,&wifi_manager_wifi_event_hadler,NULL,NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
    ESP_EVENT_ANY_ID,&wifi_manager_wifi_event_hadler,NULL,NULL));

    esp_wifi_set_mode(WIFI_MODE_STA);

    ESP_ERROR_CHECK(esp_wifi_start());

}

