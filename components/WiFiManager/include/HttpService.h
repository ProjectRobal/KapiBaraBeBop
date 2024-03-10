/*

    A file with http server related functions.

*/

#pragma once

#include "esp_http_server.h"

void start_http(void);

void stop_http(void);


//----Endpoints

esp_err_t home(httpd_req_t *req);

esp_err_t set_ssid(httpd_req_t *req);