#include "modes.h"
#include "hal/adc_types.h"
//#include "esp_log.h"

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

esp_err_t set_pin_analog_input(uint8_t pin_nr) {
    adc_atten_t adc_attention = ADC_ATTEN_DB_11;
    adc1_channel_t channel = ADC1_CHANNEL_5;

    //ADC1 config
    esp_err_t err = adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    err = adc1_config_channel_atten(channel, adc_attention);

    board.pin_functions[pin_nr] = &read_pin_analog;
    board.adc_handles[pin_nr] = channel;

    return err;
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


