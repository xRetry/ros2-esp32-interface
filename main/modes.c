#include "modes.h"
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "soc/adc_channel.h"
#include "soc/dac_channel.h"
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
    esp_err_t err = gpio_set_level(pin_nr, *val);
    return err;
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
    esp_err_t err = ESP_OK;
    if (board.adc_unit[pin_nr] == 1) *val = adc1_get_raw(board.channels[pin_nr]);
    else err = adc2_get_raw(board.channels[pin_nr], ADC_WIDTH_BIT_DEFAULT, (int*) val);
    return err;
}

esp_err_t write_pin_analog(uint8_t pin_nr, double *val) {
    dac_output_voltage(board.channels[pin_nr], *val);
    return ESP_OK;
}

esp_err_t set_pin_analog_output(uint8_t pin_nr) {
    int channel;
    switch (pin_nr) {
        case DAC_CHANNEL_1_GPIO_NUM: channel = DAC_CHANNEL_1; break;
        case DAC_CHANNEL_2_GPIO_NUM: channel = DAC_CHANNEL_2; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    dac_output_enable(channel);
    board.pin_functions[pin_nr] = &write_pin_analog;
    board.channels[pin_nr] = channel;
    return ESP_OK;
}

esp_err_t set_pin_analog_input(uint8_t pin_nr) {
    adc_atten_t adc_attention = ADC_ATTEN_DB_11;

    int adc1_channel = -1;
    switch (pin_nr) {
        case ADC1_CHANNEL_0_GPIO_NUM: adc1_channel = ADC1_CHANNEL_0; break;
        case ADC1_CHANNEL_1_GPIO_NUM: adc1_channel = ADC1_CHANNEL_1; break;
        case ADC1_CHANNEL_2_GPIO_NUM: adc1_channel = ADC1_CHANNEL_2; break;
        case ADC1_CHANNEL_3_GPIO_NUM: adc1_channel = ADC1_CHANNEL_3; break;
        case ADC1_CHANNEL_4_GPIO_NUM: adc1_channel = ADC1_CHANNEL_4; break;
        case ADC1_CHANNEL_5_GPIO_NUM: adc1_channel = ADC1_CHANNEL_5; break;
        case ADC1_CHANNEL_6_GPIO_NUM: adc1_channel = ADC1_CHANNEL_6; break;
        case ADC1_CHANNEL_7_GPIO_NUM: adc1_channel = ADC1_CHANNEL_7; break;
    }

    adc2_channel_t adc2_channel = -1;
    switch (pin_nr) {
        case ADC2_CHANNEL_0_GPIO_NUM: adc2_channel = ADC2_CHANNEL_0; break;
        case ADC2_CHANNEL_1_GPIO_NUM: adc2_channel = ADC2_CHANNEL_1; break;
        case ADC2_CHANNEL_2_GPIO_NUM: adc2_channel = ADC2_CHANNEL_2; break;
        case ADC2_CHANNEL_3_GPIO_NUM: adc2_channel = ADC2_CHANNEL_3; break;
        case ADC2_CHANNEL_4_GPIO_NUM: adc2_channel = ADC2_CHANNEL_4; break;
        case ADC2_CHANNEL_5_GPIO_NUM: adc2_channel = ADC2_CHANNEL_5; break;
        case ADC2_CHANNEL_6_GPIO_NUM: adc2_channel = ADC2_CHANNEL_6; break;
        case ADC2_CHANNEL_7_GPIO_NUM: adc2_channel = ADC2_CHANNEL_7; break;
        case ADC2_CHANNEL_8_GPIO_NUM: adc2_channel = ADC2_CHANNEL_8; break;
        case ADC2_CHANNEL_9_GPIO_NUM: adc2_channel = ADC2_CHANNEL_9; break;
    }

    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (adc1_channel != -1) {
        err = adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
        err = adc1_config_channel_atten(adc1_channel, adc_attention);

        board.pin_functions[pin_nr] = &read_pin_analog;
        board.channels[pin_nr] = adc1_channel;
        board.adc_unit[pin_nr] = 1;
    } 
    if (adc2_channel != -1) {
        err = adc2_config_channel_atten(adc2_channel, adc_attention);

        board.pin_functions[pin_nr] = &read_pin_analog;
        board.channels[pin_nr] = adc2_channel;
        board.adc_unit[pin_nr] = 2;
    } 

    return err;
}

///

esp_err_t (*const PIN_MODE_FUNCTIONS[])(uint8_t) = {
    &set_pin_disabled,
    &set_pin_digital_input,
    &set_pin_digital_output,
    &set_pin_analog_input,
    &set_pin_analog_output,
};

const pin_mode_directions_t PIN_DIRECTIONS[] = {
    DISABLED,
    INPUT,
    OUTPUT,
    INPUT,
    OUTPUT,
};

const size_t NUM_MODES = sizeof(PIN_DIRECTIONS) / sizeof(PIN_DIRECTIONS[0]);
