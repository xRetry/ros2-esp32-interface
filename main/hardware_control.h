#ifndef HARDWARE_CONTROL_H_   /* Include guard */
#define HARDWARE_CONTROL_H_   /* Include guard */

#include <pthread.h>
#include "driver/gpio.h"

#define NUM_PINS 4

typedef enum pin_mode_t {DISABLED, INPUT, OUTPUT} pin_mode_t;

typedef enum pin_type_t {DIGITAL, ANALOG} pin_type_t;

typedef struct board_t {
    gpio_num_t pins[NUM_PINS];
    pin_mode_t mode[NUM_PINS];
    pin_type_t type[NUM_PINS];
    pthread_rwlock_t lock;
} board_t;

board_t board;

void init_board();

bool set_pin_disabled(int pin_nr);

bool set_pin_digital_input(int pin_nr);

bool set_pin_digital_output(int pin_nr);

bool set_pin_analog_input(int _pin_nr);

bool set_pin_analog_output(int _pin_nr);

#endif // HARDWARE_CONTROL_H_


