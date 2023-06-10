#ifndef MODES_H_   /* Include guard */
#define MODES_H_

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/dac.h"
#include <ros2_esp32_messages/msg/pin_values.h>
#include <ros2_esp32_messages/srv/set_config.h>
#include <rclc/executor.h>

#define NUM_PINS 36

typedef ros2_esp32_messages__srv__SetConfig_Request set_config_req_t;
typedef ros2_esp32_messages__srv__SetConfig_Response set_config_rsp_t;
typedef ros2_esp32_messages__msg__PinValues pin_values_t;

typedef enum pin_mode_directions_t {
    DISABLED,
    INPUT,
    OUTPUT,
} pin_mode_directions_t;

//typedef enum pin_mode_t {
//    MODE_DISABLED = 0,
//    MODE_DIGITAL_INPUT = 1,
//    MODE_DIGITAL_OUTPUT = 2,
//    MODE_ANALOG_INPUT = 3,
//    MODE_ANALOG_OUTPUT = 4,
//} pin_mode_t;

typedef struct {
    uint32_t refresh_rate_ms;
    uint8_t pin_modes[NUM_PINS];
    esp_err_t (*pin_functions[NUM_PINS])(uint8_t, double*);
    int channels[NUM_PINS];
    uint8_t adc_unit[NUM_PINS];
    pthread_rwlock_t lock;
    esp_err_t pin_errors[NUM_PINS];
    rcl_ret_t node_error;
    rcl_ret_t transport_error;
} board_t;

extern board_t board;
extern const size_t NUM_MODES;
extern const pin_mode_directions_t PIN_DIRECTIONS[];
extern esp_err_t (*const PIN_MODE_FUNCTIONS[])(uint8_t);

#endif // MODES_H_
