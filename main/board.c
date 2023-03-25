#include "board.h"

board_t board;
pthread_rwlock_t _lock = PTHREAD_RWLOCK_INITIALIZER;

esp_err_t board_init() {
    printf("board_init\n");
    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        board.pin_modes[pin_nr] = MODE_DISABLED;
    }

    board.lock = _lock;
    return ESP_OK;
}

esp_err_t call_pin_functions(double (*vals)[NUM_PINS], pin_mode_directions_t mode_dir) {
    pthread_rwlock_rdlock(&board.lock);

    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        esp_err_t err = ESP_FAIL;
        // Check the direction of the current pin mode
        if (PIN_DIRECTIONS[board.pin_modes[pin_nr]] == mode_dir) {
            err = (*board.pin_functions[pin_nr])(
                pin_nr, 
                &(*vals)[pin_nr]
            );
        }
        if (err != ESP_OK) {
            (*vals)[pin_nr] = 0.;
        }
    }

    pthread_rwlock_unlock(&board.lock);
    return ESP_OK;
}

esp_err_t board_read(double (*vals_out)[NUM_PINS]) {
    return call_pin_functions(vals_out, INPUT);
}


esp_err_t board_write(double (*vals_in)[NUM_PINS]) {
    return call_pin_functions(vals_in, OUTPUT);
}

esp_err_t board_set_pin(set_pin_req_t *request) {
    pin_config_t *cfg = &request->new_config;
    uint8_t pin_mode = (uint8_t) cfg->pin_mode;
    uint8_t pin_nr = (uint8_t) cfg->pin_nr;

    pthread_rwlock_wrlock(&board.lock);

    // TODO: Change message to contain mode
    esp_err_t err = ESP_FAIL;
    err = PIN_MODE_FUNCTIONS[pin_mode](pin_nr);

    if (err == ESP_OK) {
        board.pin_modes[pin_nr] = pin_mode;
    }

    pthread_rwlock_unlock(&board.lock);
    return err;
}

