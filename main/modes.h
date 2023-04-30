#ifndef MODES_H_   /* Include guard */
#define MODES_H_

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/dac.h"
#include <ros2_esp32_interfaces/msg/detail/pin_config__functions.h>
#include <ros2_esp32_interfaces/msg/detail/pin_config__struct.h>
#include <ros2_esp32_interfaces/msg/detail/node_config__functions.h>
#include <ros2_esp32_interfaces/msg/detail/node_config__struct.h>
#include <ros2_esp32_interfaces/msg/detail/transport_config__functions.h>
#include <ros2_esp32_interfaces/msg/detail/transport_config__struct.h>
#include <ros2_esp32_interfaces/msg/pin_values.h>
#include <ros2_esp32_interfaces/msg/pin_config.h>
#include <ros2_esp32_interfaces/srv/set_config.h>
#include <ros2_esp32_interfaces/srv/detail/set_config__functions.h>
#include <rclc/executor.h>

#define NUM_PINS 36

typedef ros2_esp32_interfaces__srv__SetConfig_Request set_config_req_t;
typedef ros2_esp32_interfaces__srv__SetConfig_Response set_config_rsp_t;
typedef ros2_esp32_interfaces__msg__PinValues pin_values_t;
typedef ros2_esp32_interfaces__msg__PinConfig pin_config_t;
typedef ros2_esp32_interfaces__msg__NodeConfig node_config_t;
typedef ros2_esp32_interfaces__msg__TransportConfig transport_config_t;

typedef enum pin_mode_directions_t {
    DISABLED,
    INPUT,
    OUTPUT,
} pin_mode_directions_t;

typedef enum pin_mode_t {
    MODE_DISABLED = ros2_esp32_interfaces__msg__PinConfig__MODE_DISABLED,
    MODE_DIGITAL_INPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_DIGITAL_INPUT, 
    MODE_DIGITAL_OUTPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_DIGITAL_OUTPUT,
    MODE_ANALOG_INPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_ANALOG_INPUT,
    MODE_ANALOG_OUTPUT = ros2_esp32_interfaces__msg__PinConfig__MODE_ANALOG_OUTPUT,
} pin_mode_t;

typedef struct {
    bool use_wifi;
    char *wifi_ssid;
    char *wifi_pw;
    char *agent_ip;
    char *agent_port;
    char *node_name;
    char *subscriber_name;
    char *publisher_name;
    char *service_name;
    uint32_t refresh_rate_ms;
    pin_mode_t pin_modes[NUM_PINS];
    esp_err_t (*pin_functions[NUM_PINS])(uint8_t, double*);
    adc1_channel_t adc_handles[NUM_PINS];
    pthread_rwlock_t lock;
    esp_err_t pin_errors[NUM_PINS];
    rcl_ret_t node_error;
    rcl_ret_t transport_error;
} board_t;

extern board_t board;
extern const pin_mode_directions_t PIN_DIRECTIONS[5];
extern esp_err_t (*const PIN_MODE_FUNCTIONS[5])(uint8_t);

#endif // MODES_H_
