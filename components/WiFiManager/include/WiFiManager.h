#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>

#define DEFAULT_WIFI_MANAGER_AP_SSID "ESP-AP"

// because why not
#define WIFI_MANAGER_NETWORK_FILENAME "/littlefs/wifi.txt"

#define WIFIM_TASK_CORE_ID 1

#define WIFI_TASK_STACK_SIZE 32768

#define WIFI_CONNECTION_ATTEMPTS 3

// events bits definition

#define EVENT_BITS_WIFI_DISCONNECTED (1<<0)

#define EVENT_BITS_NEW_SSID (1<<1)

// ---- Function for handling reading/saving to flash

esp_err_t wifi_manager_init_littlefs(bool format_partion_on_failure,const char* partition_name);

bool wifi_manager_read_network_from_flash(char *ssid,char* password);

void wifi_manager_save_network_to_flash(const char *ssid,const char* password);


// --- WIFI Event handler

void wifi_manager_wifi_event_hadler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);

// --- General WiFi Functions

void wifi_manager_turn_on_ap(void);

void wifi_manager_stop_ap(void);

void wifi_manager_connect_to_ssid(const char* ssid,const char* pass);

void wifi_manager_init_wifi(void);

void wifi_manager_init_ap(const char* ssid,const char* psk);


void wifi_manager_new_ssid_set();

// function that initlialize manager
void wifi_manager_init(const char* AP_SSID,const char* AP_PSK);
