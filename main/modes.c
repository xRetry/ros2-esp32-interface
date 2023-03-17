#include "modes.h"


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
    adc_unit_t unit_id; 
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(
        pin_nr, 
        &unit_id, 
        &channel
    );
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(
        board.adc_handles[pin_nr], 
        channel, 
        val
    );
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t write_pin_analog(uint8_t pin_nr, double *val) {
    return ESP_FAIL;
}

esp_err_t set_pin_analog_output(uint8_t _pin_nr) {
    return ESP_FAIL;
}

esp_err_t set_pin_analog_input(uint8_t pin_nr) {
    board.pin_functions[pin_nr] = &read_pin_analog;
    adc_unit_t unit_id; 
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(
        pin_nr, 
        &unit_id, 
        &channel
    );
    if (err != ESP_OK) return err;

    // --- Init ADC unit --- //
    adc_oneshot_unit_handle_t unit_handle;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit_id,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&unit_config, &unit_handle);
    if (err != ESP_OK) return err;

    // --- Init ADC channel --- //
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    err = adc_oneshot_config_channel(
        unit_handle, 
        channel, 
        &channel_config
    );
    if (err != ESP_OK) return err;

    board.adc_handles[pin_nr] = unit_handle;

    // TODO: Calibration
    
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


