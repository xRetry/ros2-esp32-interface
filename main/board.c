#include "board.h"
#include "esp_err.h"

typedef struct board_t {
    pin_mode_t direction[NUM_PINS];
    pin_type_t type[NUM_PINS];
    esp_err_t (*io_functions[NUM_PINS])(uint8_t, double*);
    pthread_rwlock_t lock;
} board_t;

board_t board;

esp_err_t board_init() {
    printf("board_init\n");
    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        //if (pin_nr == 7) continue;
        //esp_err_t err = gpio_set_direction(pin_nr, GPIO_MODE_DISABLE);
        //if (err != ESP_OK) return err;
        board.direction[pin_nr] = DISABLED;
        board.type[pin_nr] = DIGITAL;
    }

    board.lock = PTHREAD_RWLOCK_INITIALIZER;
    return ESP_OK;
}

esp_err_t call_io_functions(double (*vals)[NUM_PINS], pin_mode_t mode) {
    pthread_rwlock_rdlock(&board.lock);

    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        esp_err_t err = ESP_FAIL;
        if (board.direction[pin_nr] == mode) {
            err = (*board.io_functions[pin_nr])(
                pin_nr, 
                vals[pin_nr]
            );
        }
        if (err != ESP_OK) {
            *vals[pin_nr] = 0.;
        }
    }

    pthread_rwlock_unlock(&board.lock);
    return ESP_OK;
}

esp_err_t board_read(double (*vals_out)[NUM_PINS]) {
    return call_io_functions(vals_out, INPUT);
}


esp_err_t board_write(double (*vals_in)[NUM_PINS]) {
    return call_io_functions(vals_in, OUTPUT);
}

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
    board.io_functions[pin_nr] = &read_pin_digital;
    return gpio_set_direction(pin_nr, GPIO_MODE_INPUT);
}

esp_err_t set_pin_digital_output(uint8_t pin_nr) {
    board.io_functions[pin_nr] = &write_pin_digital;
    return gpio_set_direction(pin_nr, GPIO_MODE_OUTPUT);
}

// --- Analog --- //

esp_err_t  set_pin_analog_input(uint8_t _pin_nr) {
    return ESP_FAIL;
}

esp_err_t set_pin_analog_output(uint8_t _pin_nr) {
    return ESP_FAIL;
}

// --- Set pin --- //

esp_err_t board_set_pin(set_pin_req_t *request) {
    pin_config_t *cfg = &request->new_config;

    pthread_rwlock_wrlock(&board.lock);

    esp_err_t err = ESP_FAIL;
    switch (request->new_config.direction) {
        case DISABLED:
        err = set_pin_disabled(cfg->pin_nr);
        break;
    case INPUT:
        switch (cfg->type) {
        case DIGITAL:
            err = set_pin_digital_input(cfg->pin_nr);
            break;
        case ANALOG:
            err = set_pin_analog_input(cfg->pin_nr);
            break;
        }
        break;
    case OUTPUT:
        switch (cfg->type) {
        case DIGITAL:
            err = set_pin_digital_output(cfg->pin_nr);
            break;
        case ANALOG:
            err = set_pin_analog_output(cfg->pin_nr);
            break;
        }
        break;
    }

    if (err == ESP_OK) {
        board.direction[cfg->pin_nr] = cfg->direction;
        board.type[cfg->pin_nr] = cfg->type;
    }

    pthread_rwlock_unlock(&board.lock);
    return err;
}

