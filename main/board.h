#ifndef BOARD_H_   /* Include guard */
#define BOARD_H_

#include <pthread.h>
#include "driver/gpio.h"
#include <ros2_esp32_interfaces/msg/detail/pin_config__functions.h>
#include <ros2_esp32_interfaces/msg/detail/pin_config__struct.h>
#include <ros2_esp32_interfaces/msg/pin_values.h>
#include <ros2_esp32_interfaces/msg/pin_config.h>
#include <ros2_esp32_interfaces/srv/set_pin.h>
#include <ros2_esp32_interfaces/srv/detail/set_pin__functions.h>
#include "esp_err.h"

#define NUM_PINS 10

typedef ros2_esp32_interfaces__srv__SetPin_Request set_pin_req_t;
typedef ros2_esp32_interfaces__msg__PinValues pin_values_t;
typedef ros2_esp32_interfaces__msg__PinConfig pin_config_t;

typedef enum pin_mode_t {
    DISABLED = ros2_esp32_interfaces__msg__PinConfig__DIR_DISABLED,
    INPUT = ros2_esp32_interfaces__msg__PinConfig__DIR_INPUT,
    OUTPUT = ros2_esp32_interfaces__msg__PinConfig__DIR_OUTPUT
} pin_mode_t;

typedef enum pin_type_t {
    DIGITAL = ros2_esp32_interfaces__msg__PinConfig__TYPE_DIGITAL, 
    ANALOG = ros2_esp32_interfaces__msg__PinConfig__TYPE_ANALOG
} pin_type_t;

typedef struct  set_pin_rsp_t {
    bool is_ok;
} set_pin_rsp_t;

esp_err_t board_init();

esp_err_t board_write(double (*vals_in)[NUM_PINS]);

esp_err_t board_read(double (*vals_out)[NUM_PINS]);

esp_err_t board_set_pin(set_pin_req_t *request);

#endif // BOARD_H_


