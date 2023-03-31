#include "modes.h"
#include "hal/adc_types.h"
#include "esp_log.h"

// --- Disable --- //

esp_err_t  set_pin_disabled(uint8_t pin_nr) {
    return gpio_set_direction(pin_nr, GPIO_MODE_DISABLE);
}

// --- Digital --- //

esp_err_t read_pin_digital(uint8_t pin_nr, double *val) {
    *val = gpio_get_level(pin_nr);
    return ESP_OK;
}

esp_err_t write_pin_digital(uint8_t pin_nr, double *val) {
    return gpio_set_level(pin_nr, *val);
}

esp_err_t  set_pin_digital_input(uint8_t pin_nr) {
    board.pin_functions[pin_nr] = &read_pin_digital;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_nr);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    return gpio_config(&io_conf);
}

esp_err_t set_pin_digital_output(uint8_t pin_nr) {
    board.pin_functions[pin_nr] = &write_pin_digital;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin_nr);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    return gpio_config(&io_conf);
}

// --- Analog --- //

esp_err_t read_pin_analog(uint8_t pin_nr, double *val) {
    *val = adc1_get_raw(board.adc_handles[pin_nr]);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(*val, &board.adc_chars[pin_nr]);
    printf("cali data: %d mV\n", voltage);
    return ESP_OK;
}

esp_err_t write_pin_analog(uint8_t pin_nr, double *val) {
    dac_output_voltage(DAC_CHANNEL_1, *val);
    return ESP_OK;
}

esp_err_t set_pin_analog_output(uint8_t pin_nr) {
    dac_output_enable(DAC_CHANNEL_1);
    board.pin_functions[pin_nr] = &write_pin_analog;
    return ESP_OK;
}

static bool adc_calibration_init(adc_atten_t adc_attention, esp_adc_cal_characteristics_t adc_chars)
{
    bool cali_enable = false;
    esp_adc_cal_value_t cali_scheme = ESP_ADC_CAL_VAL_EFUSE_VREF;
    static const char *TAG = "ADC SINGLE";

    esp_err_t ret;
    ret = esp_adc_cal_check_efuse(cali_scheme);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, adc_attention, ADC_WIDTH_BIT_DEFAULT, 0, &adc_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

esp_err_t set_pin_analog_input(uint8_t pin_nr) {
    static esp_adc_cal_characteristics_t adc_chars;
    adc_atten_t adc_attention = ADC_ATTEN_DB_11;
    bool cali_enable = adc_calibration_init(adc_attention, adc_chars);
    if (!cali_enable) {
        return ESP_FAIL;
    }
    adc1_channel_t channel = ADC1_CHANNEL_5;

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(channel, adc_attention));

    board.pin_functions[pin_nr] = &read_pin_analog;
    board.adc_handles[pin_nr] = channel;
    board.adc_chars[pin_nr] = adc_chars;

    return ESP_OK;
}

///

esp_err_t (*const PIN_MODE_FUNCTIONS[5])(uint8_t) = {
    &set_pin_disabled,
    &set_pin_digital_input,
    &set_pin_digital_output,
    &set_pin_analog_input,
    &set_pin_analog_output,
};

const pin_mode_directions_t PIN_DIRECTIONS[5] = {
    DISABLED,
    INPUT,
    OUTPUT,
    INPUT,
    OUTPUT,
};


