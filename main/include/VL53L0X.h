#pragma once

#include "i2c.h"

typedef enum
    {
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
    } vl_calibration_type_t;

typedef struct vl53l0x
{
    uint32_t measurement_timing_budget_us;
    uint8_t i2c_port;
    uint8_t stop_variable;
    uint8_t address;
    gpio_num_t gpio_xshut;
    gpio_num_t gpio_gpio1;
}vl53l0x_t;

// protected:

uint32_t vl_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

uint32_t vl_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

bool vl_perform_single_ref_calibration(vl53l0x_t* vl,vl_calibration_type_t calib_type);

bool vl_read_strobe(vl53l0x_t* vl);

bool vl_set_sequence_steps_enabled(vl53l0x_t* vl,uint8_t sequence_step);

bool vl_configure_interrupt(vl53l0x_t* vl);

bool vl_load_default_tuning_settings(vl53l0x_t* vl);

bool vl_get_spad_info_from_nvm(vl53l0x_t* vl,uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6]);

bool vl_init_config(vl53l0x_t* vl);

bool vl_data_init(vl53l0x_t* vl);

bool vl_static_init(vl53l0x_t* vl);

bool vl_perform_ref_calibration(vl53l0x_t* vl);

bool vl_set_spads_from_nvm(vl53l0x_t* vl);

uint16_t vl_decodeTimeout(uint16_t reg_val);

uint16_t vl_encodeTimeout(uint32_t timeout_mclks);

// public:

vl53l0x_t vl_new(i2c_port_t i2c_port, gpio_num_t gpio_xshut,gpio_num_t gpio_gpio1);

bool vl_init(vl53l0x_t* vl);

bool vl_setContinousMode(vl53l0x_t* vl);

bool vl_stopContinuousMode(vl53l0x_t* vl);

uint16_t vl_readContinous(vl53l0x_t* vl);

uint16_t vl_read(vl53l0x_t* vl);

bool vl_setTimingBudget(vl53l0x_t* vl,uint32_t timing);

uint32_t vl_getTimingBudget(vl53l0x_t* vl);

bool vl_setSignalRateLimit(vl53l0x_t* vl,float limit);

float vl_getSignalRateLimit(vl53l0x_t* vl);

bool vl_setVcselPulsePeriodPre(vl53l0x_t* vl,uint8_t period);

bool vl_setVcselPulsePeriodFinal(vl53l0x_t* vl,uint8_t period);

bool vl_setAdress(vl53l0x_t* vl,uint8_t address);