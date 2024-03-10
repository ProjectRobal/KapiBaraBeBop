#pragma once

#include "driver/i2c.h"

bool i2c_writeBytes(uint8_t port,uint8_t address,uint8_t _register,const uint8_t *data,uint32_t length);

bool i2c_write8(uint8_t port,uint8_t address,uint8_t _register,uint8_t data);

bool i2c_write16(uint8_t port,uint8_t address,uint8_t _register,uint16_t data);

bool i2c_write32(uint8_t port,uint8_t address,uint8_t _register,uint32_t data);


bool i2c_readBytes(uint8_t port,uint8_t address,uint8_t _register,uint8_t *data,uint32_t length);

bool i2c_read8(uint8_t port,uint8_t address,uint8_t _register,uint8_t *data);

bool i2c_read16(uint8_t port,uint8_t address,uint8_t _register,uint16_t *data);

bool i2c_read32(uint8_t port,uint8_t address,uint8_t _register,uint32_t *data);