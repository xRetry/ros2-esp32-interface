#include "hardware_control.h"

void init_board() {
    gpio_num_t pins[] = {
        GPIO_NUM_1,
        GPIO_NUM_2,
        GPIO_NUM_3,
        GPIO_NUM_4,
    };

    for (int i=0; i<NUM_PINS; i++) {
        gpio_reset_pin(board.pins[i]);
        board.pins[i] = pins[i];
        board.mode[i] = DISABLED;
        board.type[i] = DIGITAL;
    }

    board.lock = PTHREAD_RWLOCK_INITIALIZER;
}

bool set_pin_disabled(int pin_nr) {
    return gpio_set_direction(board.pins[pin_nr], GPIO_MODE_DISABLE);
}

bool set_pin_digital_input(int pin_nr) {
    return gpio_set_direction(board.pins[pin_nr], GPIO_MODE_INPUT);
}

bool set_pin_digital_output(int pin_nr) {
    return gpio_set_direction(board.pins[pin_nr], GPIO_MODE_OUTPUT);
}

bool set_pin_analog_input(int _pin_nr) {
    return false;
}

bool set_pin_analog_output(int _pin_nr) {
    return false;
}
