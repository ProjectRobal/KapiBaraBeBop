#include <stdio.h>
#include <esp_log.h>

#include "HttpService.h"

#include "WiFiManager.h"

#include "index.h"

static httpd_handle_t http_server;

const httpd_uri_t set_ssid_cfg={
    .uri="/ssid",
    .method=HTTP_POST,
    .handler=set_ssid,
    .user_ctx=NULL
};

const httpd_uri_t home_cfg={
    .uri="/",
    .method=HTTP_GET,
    .handler=home,
    .user_ctx=NULL
};

void start_http(void)
{
    httpd_config_t config= HTTPD_DEFAULT_CONFIG();

    if(httpd_start(&http_server,&config) == ESP_OK)
    {

        ESP_LOGI("WiFiManager","Starting http server");

        ESP_ERROR_CHECK(httpd_register_uri_handler(http_server,&home_cfg));
        ESP_ERROR_CHECK(httpd_register_uri_handler(http_server,&set_ssid_cfg)); 
    }
    else
    {
        ESP_LOGE("WiFiManager","Error starting server");
    }
}

void stop_http(void)
{
    httpd_stop(&http_server);
}

esp_err_t home(httpd_req_t *req)
{
    if(req->method!=HTTP_GET)   
    {
        return ESP_FAIL;
    }

    httpd_resp_set_type(req,"text/html");
    
    httpd_send(req,index_html,index_html_len);

    return ESP_OK;
}

esp_err_t set_ssid(httpd_req_t *req)
{
    if(req->method!=HTTP_POST)   
    {
        return ESP_FAIL;   
    }

    char ssid[33];
    char password[65];

    if(!req->content_len)
    {
        return ESP_FAIL;
    }

    char content[req->content_len];

    int ret = httpd_req_recv(req, content, req->content_len);

    if (ret <= 0) { 

        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
           
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, NULL);

        }
        
        return ESP_FAIL;
    }

    if( httpd_query_key_value(content,"ssid",ssid,33) !=ESP_OK )
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
        return ESP_FAIL;
    }
    if( httpd_query_key_value(content,"password",password,65) !=ESP_OK )
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
        return ESP_FAIL;
    }

    //wifi_manager_save_network_to_flash(ssid,password);

    wifi_manager_connect_to_ssid(ssid,password);

    wifi_manager_new_ssid_set();

    httpd_resp_send(req,"OK",3);

    return ESP_OK;
}