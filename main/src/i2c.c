#include "i2c.h"
#include "esp_log.h"


bool i2c_writeBytes(uint8_t port,uint8_t address,uint8_t _register,const uint8_t *data,uint32_t length)
{
    esp_err_t err=ESP_OK;

    i2c_cmd_handle_t cmd=i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd,address<<1 | I2C_MASTER_WRITE,I2C_MASTER_ACK);

    i2c_master_write_byte(cmd,_register,I2C_MASTER_ACK);

    i2c_master_write(cmd,data,length,I2C_MASTER_ACK);

    i2c_master_stop(cmd);

    err=i2c_master_cmd_begin((i2c_port_t)port,cmd,100/portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if(err!=ESP_OK)
    {

    ESP_LOGE("i2c","%s",esp_err_to_name(err));

    }

    return err==ESP_OK;
}

bool i2c_write8(uint8_t port,uint8_t address,uint8_t _register,uint8_t data)
{
    return i2c_writeBytes(port,address,_register,&data,1);
}

bool i2c_write16(uint8_t port,uint8_t address,uint8_t _register,uint16_t data)
{
    uint8_t buffer[2]; // 2
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    return i2c_writeBytes(port,address,_register,buffer,2);
}

bool i2c_write32(uint8_t port,uint8_t address,uint8_t _register,uint32_t data)
{
    uint8_t buffer[4]; // 4

    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    return i2c_writeBytes(port,address,_register,buffer,4);
}


bool i2c_readBytes(uint8_t port,uint8_t address,uint8_t _register,uint8_t *data,uint32_t length)
{

    esp_err_t err=ESP_OK;

    i2c_cmd_handle_t cmd=i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd,address<<1 | I2C_MASTER_WRITE,I2C_MASTER_ACK);

    i2c_master_write_byte(cmd,_register,I2C_MASTER_ACK);

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd,address<<1 | I2C_MASTER_READ,I2C_MASTER_ACK);

    i2c_master_read(cmd,data,length,I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    err=i2c_master_cmd_begin((i2c_port_t)port,cmd,100/portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if(err!=ESP_OK)
    {

    ESP_LOGE("i2c","%s",esp_err_to_name(err));

    }

    return err==ESP_OK;

}

bool i2c_read8(uint8_t port,uint8_t address,uint8_t _register,uint8_t *data)
{

    return i2c_readBytes(port,address,_register,data,1);
}

bool i2c_read16(uint8_t port,uint8_t address,uint8_t _register,uint16_t *data)
{
    uint8_t  buffer[2];

    bool res=i2c_readBytes(port,address,_register,buffer,2);

    *data = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return res;
}

bool i2c_read32(uint8_t port,uint8_t address,uint8_t _register,uint32_t *data)
{
    uint8_t  buffer[4];
    
    bool res= i2c_readBytes(port,address,_register,buffer,4);

    *data = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) +
             ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return res;
}