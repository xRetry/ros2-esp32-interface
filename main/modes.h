#ifndef MODES_H_   /* Include guard */
#define MODES_H_

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include <ros2_esp32_interfaces/msg/detail/pin_config__functions.h>
#include <ros2_esp32_interfaces/msg/detail/pin_config__struct.h>
#include <ros2_esp32_interfaces/msg/pin_values.h>
#include <ros2_esp32_interfaces/msg/pin_config.h>
#include <ros2_esp32_interfaces/srv/set_pin.h>
#include <ros2_esp32_interfaces/srv/detail/set_pin__functions.h>
#include "esp_err.h"

#define NUM_PINS 35

typedef ros2_esp32_interfaces__srv__SetPin_Request set_pin_req_t;
typedef ros2_esp32_interfaces__msg__PinValues pin_values_t;
typedef ros2_esp32_interfaces__msg__PinConfig pin_config_t;

typedef enum pin_mode_t {
    MODE_DISABLED = ros2_esp32_interfaces__msg__PinConfig__MODE_DISABLED,
    MODE_DIGITAL_INPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_DIGITAL_INPUT, 
    MODE_DIGITAL_OUTPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_DIGITAL_OUTPUT,
    MODE_ANALOG_INPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_ANALOG_INPUT,
    MODE_ANALOG_OUTPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_ANALOG_OUTPUT,
} pin_mode_t;

typedef enum pin_mode_directions_t {
    DISABLED,
    INPUT,
    OUTPUT,
} pin_mode_directions_t;

typedef struct board_c {
    pin_mode_t pin_modes[NUM_PINS];
    esp_err_t (*pin_functions[NUM_PINS])(uint8_t, double*);
    adc_oneshot_unit_handle_t adc_handles[NUM_PINS];
    pthread_rwlock_t lock;
} board_t;

extern board_t board;

esp_err_t set_pin_disabled(uint8_t pin_nr);
esp_err_t set_pin_digital_input(uint8_t pin_nr);
esp_err_t set_pin_digital_output(uint8_t pin_nr);
esp_err_t set_pin_analog_input(uint8_t _pin_nr);
esp_err_t set_pin_analog_output(uint8_t _pin_nr);

extern esp_err_t (*const PIN_MODE_FUNCTIONS[5])(uint8_t);
extern const pin_mode_directions_t PIN_DIRECTIONS[5];
#endif // MODES_H_
