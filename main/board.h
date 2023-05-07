#ifndef BOARD_H_   /* Include guard */
#define BOARD_H_

#include <pthread.h>
#include "esp_err.h"
#include "modes.h"

void board_init();

void board_write(double vals_in[NUM_PINS]);

void board_read(double vals_out[NUM_PINS]);

void board_set_pins(uint8_t pin_config[NUM_PINS]);

#endif // BOARD_H_


