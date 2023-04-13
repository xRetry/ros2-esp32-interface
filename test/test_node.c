#include "unity.h"
#include <string.h>
#include <pthread.h>
#include "esp_err.h"
#include <rcl/error_handling.h>

#include "mock_rclc.h"
#include "mock_allocator.h"
#include "mock_executor.h"
#include "mock_uros_network_interfaces.h"
#include "mock_subscription.h"
#include "mock_publisher.h"
#include "mock_timer.h"
#include "mock_node.h"
#include "mock_init.h"
#include "mock_service.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "mock_dac.h"
#include "mock_dac_common.h"

#include "board.h"
#include "modes.h"
#include "ros2_node.c"

board_t board;

rcl_init_options_t rcl_get_zero_initialized_init_options(void) {
  return (const rcl_init_options_t) {
           .impl = 0,
  };  
}

rcl_ret_t
rcl_init_options_init(rcl_init_options_t * init_options, rcl_allocator_t allocator) {
  return RCL_RET_OK;
}

void setUp(void) {}

void test_handle_write_pins() {
    gpio_config_IgnoreAndReturn(ESP_OK);

    uint8_t pin_modes[NUM_PINS];
    pin_config_t pin_config;
    for (int i=0; i<NUM_PINS; i++) pin_config.pin_modes[i] = MODE_DIGITAL_OUTPUT;

    board_init();
    board_set_pins(&pin_config);

    pin_values_t msg; 
    for (int i=0; i<NUM_PINS; i++) {
        msg.values[i] = i;
        gpio_set_level_ExpectAndReturn(i, i, ESP_OK);
    }

    handle_write_pins((void*) &msg);
}

void test_handle_read_pins() {
    gpio_config_IgnoreAndReturn(ESP_OK);

    uint8_t pin_modes[NUM_PINS];
    pin_config_t pin_config;
    for (int i=0; i<NUM_PINS; i++) pin_config.pin_modes[i] = MODE_DIGITAL_INPUT;

    board_init();
    board_set_pins(&pin_config);

    pin_values_t msg; 
    for (int i=0; i<NUM_PINS; i++) {
        msg.values[i] = i;
        gpio_get_level_ExpectAndReturn(i, i);
    }

    rcl_publish_ExpectAndReturn(&board.publisher, &msg, NULL, RCL_RET_TIMEOUT);
    rcl_timer_t timer;
    handle_read_pins(&timer, 0);

    TEST_ASSERT_EQUAL_INT(RCL_RET_TIMEOUT, board.node_error);
}

void test_handle_set_config() {
    board_init();

    set_config_req_t req;
    set_config_rsp_t rsp;

    req.change_pins = true;
    req.change_node = false;
    req.change_transport = false;
    req.new_node_config.refresh_rate_ms = 999;
    req.new_node_config.node_name.data = "test_node";
    req.new_node_config.publisher_name.data = "test_pub";
    req.new_node_config.subscriber_name.data = "test_sub";
    req.new_node_config.service_name.data = "test_srv";
    req.new_transport_config.use_wifi = false;
    req.new_transport_config.agent_ip.data = "10.0";
    req.new_transport_config.agent_port.data = "0000";
    req.new_transport_config.wifi_ssid.data = "abcd";
    req.new_transport_config.wifi_pw.data = "pwd";
    
    for (int i=0; i<NUM_PINS; i++) {
        req.new_pin_config.pin_modes[i] = MODE_DIGITAL_INPUT;
        gpio_config_ExpectAnyArgsAndReturn(ESP_OK);
    }

    rcl_subscription_fini_ExpectAnyArgsAndReturn(RCL_RET_OK);
    rcl_publisher_fini_ExpectAnyArgsAndReturn(RCL_RET_OK);
    //rcl_service_fini_ExpectAnyArgsAndReturn(RCL_RET_OK);
    //rcl_node_fini_ExpectAnyArgsAndReturn(RCL_RET_OK);

    handle_set_config(&req, (void*) &rsp);

    for (int i=0; i<NUM_PINS; i++) {
        TEST_ASSERT_EQUAL_INT(MODE_DIGITAL_INPUT, board.pin_modes[i]);
    }

    TEST_ASSERT_FLOAT_WITHIN(1e-5, req.new_node_config.refresh_rate_ms, rsp.active_node_config.refresh_rate_ms);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_node_config.node_name.data, rsp.active_node_config.node_name.data, 10);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_node_config.publisher_name.data, rsp.active_node_config.publisher_name.data, 9);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_node_config.subscriber_name.data, rsp.active_node_config.subscriber_name.data, 9);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_node_config.service_name.data, rsp.active_node_config.service_name.data, 9);
    TEST_ASSERT_FALSE(rsp.active_transport_config.use_wifi);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_transport_config.agent_ip.data, rsp.active_transport_config.agent_ip.data, 5);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_transport_config.agent_port.data, rsp.active_transport_config.agent_port.data, 5);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_transport_config.wifi_ssid.data, rsp.active_transport_config.wifi_ssid.data, 5);
    TEST_ASSERT_EQUAL_STRING_LEN(req.new_transport_config.wifi_pw.data, rsp.active_transport_config.wifi_pw.data, 4);
}

void test_init_node(void) {

    rcutils_allocator_t allocator;
    rcutils_get_default_allocator_IgnoreAndReturn(allocator);
    uros_network_interface_initialize_IgnoreAndReturn(ESP_OK);

    board_init();
    rcl_ret_t err = node_try_init();
}
