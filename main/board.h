#ifndef BOARD_H_   /* Include guard */
#define BOARD_H_

#include <pthread.h>
#include "esp_err.h"
#include "modes.h"

esp_err_t board_init();

esp_err_t board_write(double (*vals_in)[NUM_PINS]);

esp_err_t board_read(double (*vals_out)[NUM_PINS]);

esp_err_t board_set_pin(set_pin_req_t *request);

#endif // BOARD_H_


