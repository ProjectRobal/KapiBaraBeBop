#include "VL53L0X.h"

#include "vl53l0x_regs.h"

#include "esp_log.h"

#include "esp_timer.h"


// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


vl53l0x_t vl_new(i2c_port_t i2c_port, gpio_num_t gpio_xshut,gpio_num_t gpio_gpio1)
{
  vl53l0x_t out={
        .address=VL53L0X_ADRESS,
        .stop_variable = 0,
        .i2c_port = i2c_port,
        .gpio_xshut=gpio_xshut,
        .gpio_gpio1=gpio_gpio1

    };

    return out;
}

bool vl_read_strobe(vl53l0x_t* vl)
{
    bool success = false;
    uint8_t strobe = 0;
    if (!i2c_write8(vl->i2c_port,vl->address,0x83, 0x00)) {
        return false;
    }
    do {
        success = i2c_read8(vl->i2c_port,vl->address,0x83, &strobe);
    } while (success && (strobe == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write8(vl->i2c_port,vl->address,0x83, 0x01)) {
        return false;
    }
    return true;
}

bool vl_get_spad_info_from_nvm(vl53l0x_t* vl,uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
    bool success = false;
    uint8_t tmp_data8 = 0;
    uint32_t tmp_data32 = 0;

    /* Setup to read from NVM */
    success  = i2c_write8(vl->i2c_port,vl->address,0x80, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x06);
    success &= i2c_read8(vl->i2c_port,vl->address,0x83, &tmp_data8);
    success &= i2c_write8(vl->i2c_port,vl->address,0x83, tmp_data8 | 0x04);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x07);
    success &= i2c_write8(vl->i2c_port,vl->address,0x81, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x01);
    if (!success) {
      return false;
    }

    /* Get the SPAD count and type */
    success &= i2c_write8(vl->i2c_port,vl->address,0x94, 0x6b);
    if (!success) {
        return false;
    }
    if (!vl_read_strobe(vl)) {
        return false;
    }
    success &= i2c_read32(vl->i2c_port,vl->address,0x90, &tmp_data32);
    if (!success) {
        return false;
    }
    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;


    /* Restore after reading from NVM */
    success &=i2c_write8(vl->i2c_port,vl->address,0x81, 0x00);
    success &=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x06);
    success &=i2c_read8(vl->i2c_port,vl->address,0x83, &tmp_data8);
    success &=i2c_write8(vl->i2c_port,vl->address,0x83, tmp_data8 & 0xfb);
    success &=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &=i2c_write8(vl->i2c_port,vl->address,0x00, 0x01);
    success &=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &=i2c_write8(vl->i2c_port,vl->address,0x80, 0x00);

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from vl register
     * instead of reading it from the NVM. */
    if (!i2c_readBytes(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, good_spad_map, 6)) {
        return false;
    }
    return success;
}

bool vl_configure_interrupt(vl53l0x_t* vl)
{
    /* Interrupt on new sample ready */
    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high = 0;
    if (!i2c_read8(vl->i2c_port,vl->address,REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high)) {
        return false;
    }
    gpio_hv_mux_active_high &= ~0x10;
    if (!i2c_write8(vl->i2c_port,vl->address,REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high)) {
        return false;
    }

    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }
    return true;
}

bool vl_set_sequence_steps_enabled(vl53l0x_t* vl,uint8_t sequence_step)
{
    return i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}


bool vl_load_default_tuning_settings(vl53l0x_t* vl)
{
    bool success = i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x09, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x10, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x11, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x24, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x25, 0xFF);
    success &= i2c_write8(vl->i2c_port,vl->address,0x75, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x4E, 0x2C);
    success &= i2c_write8(vl->i2c_port,vl->address,0x48, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x30, 0x20);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x30, 0x09);
    success &= i2c_write8(vl->i2c_port,vl->address,0x54, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x31, 0x04);
    success &= i2c_write8(vl->i2c_port,vl->address,0x32, 0x03);
    success &= i2c_write8(vl->i2c_port,vl->address,0x40, 0x83);
    success &= i2c_write8(vl->i2c_port,vl->address,0x46, 0x25);
    success &= i2c_write8(vl->i2c_port,vl->address,0x60, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x27, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x50, 0x06);
    success &= i2c_write8(vl->i2c_port,vl->address,0x51, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x52, 0x96);
    success &= i2c_write8(vl->i2c_port,vl->address,0x56, 0x08);
    success &= i2c_write8(vl->i2c_port,vl->address,0x57, 0x30);
    success &= i2c_write8(vl->i2c_port,vl->address,0x61, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x62, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x64, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x65, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x66, 0xA0);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x22, 0x32);
    success &= i2c_write8(vl->i2c_port,vl->address,0x47, 0x14);
    success &= i2c_write8(vl->i2c_port,vl->address,0x49, 0xFF);
    success &= i2c_write8(vl->i2c_port,vl->address,0x4A, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x7A, 0x0A);
    success &= i2c_write8(vl->i2c_port,vl->address,0x7B, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x78, 0x21);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x23, 0x34);
    success &= i2c_write8(vl->i2c_port,vl->address,0x42, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x44, 0xFF);
    success &= i2c_write8(vl->i2c_port,vl->address,0x45, 0x26);
    success &= i2c_write8(vl->i2c_port,vl->address,0x46, 0x05);
    success &= i2c_write8(vl->i2c_port,vl->address,0x40, 0x40);
    success &= i2c_write8(vl->i2c_port,vl->address,0x0E, 0x06);
    success &= i2c_write8(vl->i2c_port,vl->address,0x20, 0x1A);
    success &= i2c_write8(vl->i2c_port,vl->address,0x43, 0x40);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x34, 0x03);
    success &= i2c_write8(vl->i2c_port,vl->address,0x35, 0x44);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x31, 0x04);
    success &= i2c_write8(vl->i2c_port,vl->address,0x4B, 0x09);
    success &= i2c_write8(vl->i2c_port,vl->address,0x4C, 0x05);
    success &= i2c_write8(vl->i2c_port,vl->address,0x4D, 0x04);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x44, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x45, 0x20);
    success &= i2c_write8(vl->i2c_port,vl->address,0x47, 0x08);
    success &= i2c_write8(vl->i2c_port,vl->address,0x48, 0x28);
    success &= i2c_write8(vl->i2c_port,vl->address,0x67, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x70, 0x04);
    success &= i2c_write8(vl->i2c_port,vl->address,0x71, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x72, 0xFE);
    success &= i2c_write8(vl->i2c_port,vl->address,0x76, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x77, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x0D, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x01, 0xF8);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x8E, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x00);
    return success;
}

bool vl_set_spads_from_nvm(vl53l0x_t* vl)
{

    uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t spads_enabled_count = 0;
    uint8_t spads_to_enable_count = 0;
    uint8_t spad_type = 0;
    volatile uint32_t total_val = 0;

    if (!vl_get_spad_info_from_nvm(vl,&spads_to_enable_count, &spad_type, good_spad_map)) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        total_val += good_spad_map[i];
    }

    bool success = i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT);
    if (!success) {
        return false;
    }

    uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

    /* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
     * The subset should only have the number of type enabled as suggested by the reading from
     * the NVM (spads_to_enable_count and spad_type). */
    for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
        for (int column = 0; column < SPAD_ROW_SIZE; column++) {
            int index = (row * SPAD_ROW_SIZE) + column;
            if (index >= SPAD_MAX_COUNT) {
                return false;
            }
            if (spads_enabled_count == spads_to_enable_count) {
                /* We are done */
                break;
            }
            if (index < offset) {
                continue;
            }
            if ((good_spad_map[row] >> column) & 0x1) {
                spad_map[row] |= (1 << column);
                spads_enabled_count++;
            }
        }
        if (spads_enabled_count == spads_to_enable_count) {
            /* To avoid looping unnecessarily when we are already done. */
            break;
        }
    }

    if (spads_enabled_count != spads_to_enable_count) {
        return false;
    }

    /* Write the new SPAD configuration */
    if (!i2c_writeBytes(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, SPAD_MAP_ROW_COUNT)) {
        return false;
    }

    
    return true;
}


bool vl_data_init(vl53l0x_t* vl)
{
    /* Set 2v8 mode */
    uint8_t vhv_config_scl_sda = 0;
    if (!i2c_read8(vl->i2c_port,vl->address,REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda)) {
        return false;
    }
    vhv_config_scl_sda |= 0x01;
    if (!i2c_write8(vl->i2c_port,vl->address,REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda)) {
        return false;
    }

    bool success;

    /* Set I2C standard mode */
    success = i2c_write8(vl->i2c_port,vl->address,0x88, 0x00);

    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x00);
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    success &= i2c_read8(vl->i2c_port,vl->address,0x91, &vl->stop_variable);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x00);

    return success;
}

bool vl_static_init(vl53l0x_t* vl)
{
    if (!vl_set_spads_from_nvm(vl)) {
        ESP_LOGE("TOF","Set spads failed!");
        return false;
    }

    if (!vl_load_default_tuning_settings(vl)) {
        ESP_LOGE("TOF","Load defualut tuning settings failed!");
        return false;
    }

    if (!vl_configure_interrupt(vl)) {
        ESP_LOGE("TOF","Configure interrupt failed!");
        return false;
    }

    if (!vl_set_sequence_steps_enabled(vl,0xE8)) {

        ESP_LOGE("TOF","Set sequence steps enabled failed!");
        return false;
    }

    return true;
}

bool vl_perform_single_ref_calibration(vl53l0x_t* vl,vl_calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }
    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG, sequence_config)) {
        return false;
    }
    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSRANGE_START, sysrange_start)) {
        return false;
    }
    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    bool success = false;

    // here it blocks
    do {
        success = i2c_read8(vl->i2c_port,vl->address,REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSRANGE_START, 0x00)) {
        return false;
    }
    return true;
}

bool vl_perform_ref_calibration(vl53l0x_t* vl)
{
    // here it hangs
    ESP_LOGE("Sensors","CALIBRATION_TYPE_VHV");
    if (!vl_perform_single_ref_calibration(vl,CALIBRATION_TYPE_VHV)) {
        return false;
    }
    ESP_LOGE("Sensors","CALIBRATION_TYPE_PHASE");
    if (!vl_perform_single_ref_calibration(vl,CALIBRATION_TYPE_PHASE)) {
        return false;
    }
    
    /* Restore sequence steps enabled */
    if (!vl_set_sequence_steps_enabled(vl,0xE8)) {
        return false;
    }
    return true;
}

bool vl_init_config(vl53l0x_t* vl)
{
    if (!vl_data_init(vl)) {
        ESP_LOGE("TOF","Data init failed!");
        return false;
    }
    if (!vl_static_init(vl)) {
        ESP_LOGE("TOF","Static init failed!");
        return false;
    }
    if (!vl_perform_ref_calibration(vl)) {
        ESP_LOGE("TOF","Calibration failed!");
        return false;
    }

    vl->measurement_timing_budget_us=vl_getTimingBudget(vl);

    return true;
}

bool vl_init(vl53l0x_t* vl)
{

    if (!vl_init_config(vl)) {
        return false;
    }

    return true;

}

bool vl_setContinousMode(vl53l0x_t* vl)
{
  bool res=i2c_write8(vl->i2c_port,vl->address,0x80,0x01);
  res&=i2c_write8(vl->i2c_port,vl->address,0xFF,0x01);
  res&=i2c_write8(vl->i2c_port,vl->address,0x00,0x00);
  res&=i2c_write8(vl->i2c_port,vl->address,0x91,vl->stop_variable);
  res&=i2c_write8(vl->i2c_port,vl->address,0x00,0x01);
  res&=i2c_write8(vl->i2c_port,vl->address,0xFF,0x00);
  res&=i2c_write8(vl->i2c_port,vl->address,0x80,0x00);


  res&=i2c_write8(vl->i2c_port,vl->address,REG_SYSRANGE_START,0x02);

  return res;

}

bool vl_stopContinuousMode(vl53l0x_t* vl)
{
  bool res=i2c_write8(vl->i2c_port,vl->address,REG_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  res&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
  res&=i2c_write8(vl->i2c_port,vl->address,0x00, 0x00);
  res&=i2c_write8(vl->i2c_port,vl->address,0x91, 0x00);
  res&=i2c_write8(vl->i2c_port,vl->address,0x00, 0x01);
  res&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);

  return res;
}

uint16_t vl_readContinous(vl53l0x_t* vl)
{

uint32_t timer=esp_timer_get_time()/1000;

uint8_t reg=0x00;

  do
  {
    if ((esp_timer_get_time()/1000)-timer > 50)
    {
      return 65535;
    }
    if(!i2c_read8(vl->i2c_port,vl->address,REG_RESULT_INTERRUPT_STATUS,&reg))
    {
      return 65534;
    }
  }
  while(!(reg&0x07));

  uint16_t output=0x00;

  if(!i2c_read16(vl->i2c_port,vl->address,REG_RESULT_RANGE_STATUS+10,&output))
  {
    return 65534;
  }

  if(!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_INTERRUPT_CLEAR,0x01))
  {
    return 65534;
  }

  return output;

}

bool vl_setAdress(vl53l0x_t* vl,uint8_t address)
{
    if(i2c_write8(vl->i2c_port,vl->address,REG_SLAVE_DEVICE_ADDRESS, address & 0x7F))
    {   
        vl->address=address;
        return true;
    }
    return false;
}

uint16_t vl_decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t vl_encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t vl_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

uint32_t vl_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint32_t vl_getTimingBudget(vl53l0x_t* vl)
{
  bool tcc, msrc, dss, pre_range, final_range;
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;

  uint8_t sequence_config = 0; 
  
  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  tcc          = (sequence_config >> 4) & 0x1;
  dss          = (sequence_config >> 3) & 0x1;
  msrc         = (sequence_config >> 2) & 0x1;
  pre_range    = (sequence_config >> 6) & 0x1;
  final_range  = (sequence_config >> 7) & 0x1;

  if(!i2c_read8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&pre_range_vcsel_period_pclks))
  {
    return false;
  }

  pre_range_vcsel_period_pclks=decodeVcselPeriod(pre_range_vcsel_period_pclks);

  if(!i2c_read8(vl->i2c_port,vl->address,REG_MSRC_CONFIG_TIMEOUT_MACROP,(uint8_t*)&msrc_dss_tcc_mclks))
  {
    return false;
  }

  msrc_dss_tcc_mclks+=1;

  msrc_dss_tcc_us =
    vl_timeoutMclksToMicroseconds(msrc_dss_tcc_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&pre_range_mclks))
  {
    return false;
  }

  pre_range_mclks =
    vl_decodeTimeout(pre_range_mclks);

  pre_range_us =
    vl_timeoutMclksToMicroseconds(pre_range_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&final_range_vcsel_period_pclks))
  {
    return false;
  }

  final_range_vcsel_period_pclks=decodeVcselPeriod(final_range_vcsel_period_pclks);



  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&final_range_mclks))
  {
    return false;
  }

  final_range_mclks =
    vl_decodeTimeout(final_range_mclks);


  if (pre_range)
  {
    final_range_mclks -= pre_range_mclks;
  }

  final_range_us =
    vl_timeoutMclksToMicroseconds(final_range_mclks,
                               final_range_vcsel_period_pclks);

  if (tcc)
  {
    budget_us += (msrc_dss_tcc_us + TccOverhead);
  }

  if (dss)
  {
    budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
  }
  else if (msrc)
  {
    budget_us += (msrc_dss_tcc_us + MsrcOverhead);
  }

  if (pre_range)
  {
    budget_us += (pre_range_us + PreRangeOverhead);
  }

  if (final_range)
  {
    budget_us += (final_range_us + FinalRangeOverhead);
  }

  vl->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

bool vl_setTimingBudget(vl53l0x_t* vl,uint32_t timing)
{
  bool tcc, msrc, dss, pre_range, final_range;
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  uint8_t sequence_config = 0; 
  
  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  tcc          = (sequence_config >> 4) & 0x1;
  dss          = (sequence_config >> 3) & 0x1;
  msrc         = (sequence_config >> 2) & 0x1;
  pre_range    = (sequence_config >> 6) & 0x1;
  final_range  = (sequence_config >> 7) & 0x1;

  if(!i2c_read8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&pre_range_vcsel_period_pclks))
  {
    return false;
  }

  pre_range_vcsel_period_pclks=decodeVcselPeriod(pre_range_vcsel_period_pclks);

  if(!i2c_read8(vl->i2c_port,vl->address,REG_MSRC_CONFIG_TIMEOUT_MACROP,(uint8_t*)&msrc_dss_tcc_mclks))
  {
    return false;
  }

  msrc_dss_tcc_mclks+=1;

  msrc_dss_tcc_us =
    vl_timeoutMclksToMicroseconds(msrc_dss_tcc_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&pre_range_mclks))
  {
    return false;
  }

  pre_range_mclks =
    vl_decodeTimeout(pre_range_mclks);

  pre_range_us =
    vl_timeoutMclksToMicroseconds(pre_range_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&final_range_vcsel_period_pclks))
  {
    return false;
  }

  final_range_vcsel_period_pclks=decodeVcselPeriod(final_range_vcsel_period_pclks);



  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&final_range_mclks))
  {
    return false;
  }

  final_range_mclks = vl_decodeTimeout(final_range_mclks);


  if (pre_range)
  {
    final_range_mclks -= pre_range_mclks;
  }

  //final_range_us =timeoutMclksToMicroseconds(final_range_mclks,final_range_vcsel_period_pclks);


if (tcc)
  {
    used_budget_us += (msrc_dss_tcc_us + TccOverhead);
  }

  if (dss)
  {
    used_budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
  }
  else if (msrc)
  {
    used_budget_us += (msrc_dss_tcc_us + MsrcOverhead);
  }

  if (pre_range)
  {
    used_budget_us += (pre_range_us + PreRangeOverhead);
  }

  if (final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > timing)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = timing - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do vl both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      vl_timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 final_range_vcsel_period_pclks);

    if (pre_range)
    {
      final_range_timeout_mclks += pre_range_mclks;
    }

    

    if(!i2c_write16(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl_encodeTimeout(final_range_timeout_mclks)))
    {
        return false;
    }
    vl->measurement_timing_budget_us = timing; // store for internal reuse
  }

 
  return true;
}


uint16_t vl_read(vl53l0x_t* vl)
{
    bool success = i2c_write8(vl->i2c_port,vl->address,0x80, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x91, vl->stop_variable);
    success &= i2c_write8(vl->i2c_port,vl->address,0x00, 0x01);
    success &= i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
    success &= i2c_write8(vl->i2c_port,vl->address,0x80, 0x00);
    if (!success) {
        return 8193;
    }

    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSRANGE_START, 0x01)) {
        return 8194;
    }

    uint8_t sysrange_start = 0;
    do {
        success = i2c_read8(vl->i2c_port,vl->address,REG_SYSRANGE_START, &sysrange_start);
    } while (success && (sysrange_start & 0x01));
    if (!success) {
        return 8195;
    }

    uint8_t interrupt_status = 0;
    do {
        success = i2c_read8(vl->i2c_port,vl->address,REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return 8196;
    }

    uint16_t range=0;

    if (!i2c_read16(vl->i2c_port,vl->address,REG_RESULT_RANGE_STATUS + 10, &range)) {
        return 8197;
    }

    if (!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return 8198;
    }

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (range == 8190 || range == 8191) {
        range = 8190;
    }

    return range;   
}

bool vl_setVcselPulsePeriodPre(vl53l0x_t* vl,uint8_t period)
{

  uint8_t vcsel_period_reg = encodeVcselPeriod(period);

  bool   pre_range;
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us;

  uint8_t sequence_config = 0; 
  
  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  pre_range    = (sequence_config >> 6) & 0x1;

  if(!i2c_read8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&pre_range_vcsel_period_pclks))
  {
    return false;
  }

  pre_range_vcsel_period_pclks=decodeVcselPeriod(pre_range_vcsel_period_pclks);

  if(!i2c_read8(vl->i2c_port,vl->address,REG_MSRC_CONFIG_TIMEOUT_MACROP,(uint8_t*)&msrc_dss_tcc_mclks))
  {
    return false;
  }

  msrc_dss_tcc_mclks+=1;

  msrc_dss_tcc_us =
    vl_timeoutMclksToMicroseconds(msrc_dss_tcc_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&pre_range_mclks))
  {
    return false;
  }

  pre_range_mclks =
    vl_decodeTimeout(pre_range_mclks);

  pre_range_us =
    vl_timeoutMclksToMicroseconds(pre_range_mclks,
                               pre_range_vcsel_period_pclks);


  if(!i2c_read8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&final_range_vcsel_period_pclks))
  {
    return false;
  }

  final_range_vcsel_period_pclks=decodeVcselPeriod(final_range_vcsel_period_pclks);



  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&final_range_mclks))
  {
    return false;
  }

  final_range_mclks =
    vl_decodeTimeout(final_range_mclks);


  if (pre_range)
  {
    final_range_mclks -= pre_range_mclks;
  }

  //final_range_us =timeoutMclksToMicroseconds(final_range_mclks, final_range_vcsel_period_pclks);


  //

  // "Set phase check limits"
    switch (period)
    {
      case 12:
        if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18))
        {
            return false;
        }
        break;

      case 14:
        if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30))
        {
            return false;
        }
        break;

      case 16:
        if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40))
        {
            return false;
        }
        break;

      case 18:
        if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50))
        {
            return false;
        }
        break;

      default:
        // invalid period
        return false;
    }

    if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,0x08))
    {
        return false;
    }

    // apply new VCSEL period

    if(!i2c_write8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,vcsel_period_reg))
    {
        return false;
    }

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      vl_timeoutMicrosecondsToMclks(pre_range_us, period);

    if(!i2c_write16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl_encodeTimeout(new_pre_range_timeout_mclks)))
    {
        return false;
    }
    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      vl_timeoutMicrosecondsToMclks(msrc_dss_tcc_us, period);

    if(!i2c_write8(vl->i2c_port,vl->address,REG_MSRC_CONFIG_TIMEOUT_MACROP,(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1)))
    {
        return false;
    }

    // "Finally, the timing budget must be re-applied"

  if(!vl_setTimingBudget(vl,vl->measurement_timing_budget_us))
  {
    return false;
  }

  // "Perform the phase calibration. vl is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin


  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  /*if(!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,0x02))
  {
    return false;
  }
  */
 
  if(!vl_perform_single_ref_calibration(vl,CALIBRATION_TYPE_PHASE))
  {
    return false;
  }

  if(!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,sequence_config))
  {
    return false;
  }

  // VL53L0X_perform_phase_calibration() end

    return true;
}

bool vl_setVcselPulsePeriodFinal(vl53l0x_t* vl,uint8_t period)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period);

  bool    pre_range;
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t final_range_us;

  uint8_t sequence_config = 0; 
  
  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  
  pre_range    = (sequence_config >> 6) & 0x1;

  if(!i2c_read8(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&pre_range_vcsel_period_pclks))
  {
    return false;
  }

  pre_range_vcsel_period_pclks=decodeVcselPeriod(pre_range_vcsel_period_pclks);

  if(!i2c_read8(vl->i2c_port,vl->address,REG_MSRC_CONFIG_TIMEOUT_MACROP,(uint8_t*)&msrc_dss_tcc_mclks))
  {
    return false;
  }

  msrc_dss_tcc_mclks+=1;


  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&pre_range_mclks))
  {
    return false;
  }

  pre_range_mclks =
    vl_decodeTimeout(pre_range_mclks);

  if(!i2c_read8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,(uint8_t*)&final_range_vcsel_period_pclks))
  {
    return false;
  }

  final_range_vcsel_period_pclks=decodeVcselPeriod(final_range_vcsel_period_pclks);



  if(!i2c_read16(vl->i2c_port,vl->address,REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,&final_range_mclks))
  {
    return false;
  }

  final_range_mclks =
    vl_decodeTimeout(final_range_mclks);


  if (pre_range)
  {
    final_range_mclks -= pre_range_mclks;
  }

  final_range_us =
    vl_timeoutMclksToMicroseconds(final_range_mclks,
                               final_range_vcsel_period_pclks);


  //

  bool states=true;

  switch (period)
    {
      case 8:
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_LIM, 0x30);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
        break;

      case 10:
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_LIM, 0x20);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
        break;

      case 12:
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_LIM, 0x20);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
        break;

      case 14:
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x01);
        states&=i2c_write8(vl->i2c_port,vl->address,REG_ALGO_PHASECAL_LIM, 0x20);
        states&=i2c_write8(vl->i2c_port,vl->address,0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    if(!states)
    {
        return false;
    }

    // apply new VCSEL period
    if(!i2c_write8(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg))
    {
        return false;
    }

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do vl both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      vl_timeoutMicrosecondsToMclks(final_range_us, period);

    if (pre_range)
    {
      new_final_range_timeout_mclks += pre_range_mclks;
    }
    
    if(!i2c_write16(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl_encodeTimeout(new_final_range_timeout_mclks)))
    {
        return false;
    }

    // "Finally, the timing budget must be re-applied"

  if(!vl_setTimingBudget(vl,vl->measurement_timing_budget_us))
  {
    return false;
  }

  // "Perform the phase calibration. vl is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin


  if(!i2c_read8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,&sequence_config))
  {
    return false;
  }

  /*if(!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,0x02))
  {
    return false;
  }
  */
 
  if(!vl_perform_single_ref_calibration(vl,CALIBRATION_TYPE_PHASE))
  {
    return false;
  }

  if(!i2c_write8(vl->i2c_port,vl->address,REG_SYSTEM_SEQUENCE_CONFIG,sequence_config))
  {
    return false;
  }

  // VL53L0X_perform_phase_calibration() end

    return true;
}

bool vl_setSignalRateLimit(vl53l0x_t* vl,float limit)
{
    if (limit < 0 || limit > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  return i2c_write16(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit * (1 << 7));
  
}

float vl_getSignalRateLimit(vl53l0x_t* vl)
{
    uint16_t output;

    i2c_read16(vl->i2c_port,vl->address,REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,&output);

    return (float)(output/(1<<7));
}
