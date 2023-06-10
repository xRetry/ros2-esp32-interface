#include "board.h"
#include "modes.h"
#include <string.h>

board_t board;
pthread_rwlock_t _lock = PTHREAD_RWLOCK_INITIALIZER;

void board_init() {
    printf("board_init\n");

    board.refresh_rate_ms = 1000;

    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        board.pin_errors[pin_nr] = ESP_OK;
        board.pin_modes[pin_nr] = MODE_DISABLED;
    }

    board.lock = _lock;
}

void call_pin_functions(double vals[NUM_PINS], pin_mode_directions_t mode_dir) {
    pthread_rwlock_rdlock(&board.lock);

    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        if (board.pin_modes[pin_nr] == MODE_DISABLED) {
            vals[pin_nr] = 0.;
            continue;
        }

        board.pin_errors[pin_nr] = ESP_FAIL;

        // Only call function if the direction is correct
        if (PIN_DIRECTIONS[board.pin_modes[pin_nr]] == mode_dir) {
            board.pin_errors[pin_nr] = (*board.pin_functions[pin_nr])(
                pin_nr, 
                &vals[pin_nr]
            );
        }
        if (board.pin_errors[pin_nr] != ESP_OK) {
            vals[pin_nr] = 0.;
        }
    }

    pthread_rwlock_unlock(&board.lock);
}

void board_read(double vals_out[NUM_PINS]) {
    printf(">>> Board read\n");
    call_pin_functions(vals_out, INPUT);
}


void board_write(double vals_in[NUM_PINS]) {
    printf(">>> Board write\n");
    call_pin_functions(vals_in, OUTPUT);
}

void board_set_pins(uint8_t pin_modes[NUM_PINS]) {
    for (int pin_nr=0; pin_nr<NUM_PINS; pin_nr++) {
        if (pin_nr == 7 || pin_nr == 8 || pin_nr == 11) continue;
        if (pin_modes[pin_nr] == board.pin_modes[pin_nr]) continue;

        printf("pin: %d, new mode: %d, old mode: %d\n", pin_nr, pin_modes[pin_nr], board.pin_modes[pin_nr]);


        board.pin_errors[pin_nr] = PIN_MODE_FUNCTIONS[pin_modes[pin_nr]](pin_nr);
        if (board.pin_errors[pin_nr] == ESP_OK) {
            board.pin_modes[pin_nr] = pin_modes[pin_nr];
        } else {
            board.pin_modes[pin_nr] = MODE_DISABLED;
        }
    }
}

