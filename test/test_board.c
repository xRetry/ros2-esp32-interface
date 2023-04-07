#include "unity.h"
#include <pthread.h>
#include "esp_err.h"
#include "board.h"
#include "modes.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "mock_dac.h"
#include "mock_dac_common.h"


double pin_values[NUM_PINS];

int callback_gpio_get_level(gpio_num_t pin_nr, int num_calls) {
    return pin_values[pin_nr];
}

esp_err_t callback_gpio_set_level(gpio_num_t pin_nr, uint32_t val, int num_calls) {
    pin_values[pin_nr] = val;
    return ESP_OK;
}

void setUp(void) {
    gpio_get_level_StubWithCallback(&callback_gpio_get_level);
    gpio_set_level_StubWithCallback(&callback_gpio_set_level);
    gpio_config_IgnoreAndReturn(ESP_OK);
}

void test_init_board(void) {
    board_init();

    TEST_ASSERT_EQUAL_STRING_LEN("127.0.0.1", board.agent_ip, 10);
    TEST_ASSERT_EQUAL_STRING_LEN("8888", board.agent_port, 5);
    TEST_ASSERT_EQUAL_STRING_LEN("esp32_interface", board.node_name, 16);
    TEST_ASSERT_EQUAL_STRING_LEN("esp32_write_pins", board.subscriber_name, 17);
    TEST_ASSERT_EQUAL_STRING_LEN("esp32_read_pins", board.publisher_name, 16);
    TEST_ASSERT_EQUAL_STRING_LEN("esp32_set_config", board.service_name, 17);
    TEST_ASSERT_EQUAL_STRING_LEN("ssid", board.wifi_ssid, 5);
    TEST_ASSERT_EQUAL_STRING_LEN("wifi_pw", board.wifi_pw, 8);
    TEST_ASSERT_TRUE(board.use_wifi);
    TEST_ASSERT_EQUAL_UINT32(1000, board.refresh_rate_ms);

    for (int i=0; i<NUM_PINS; i++) {
        TEST_ASSERT_TRUE(board.pin_modes[i] == MODE_DISABLED);
    }
}

void test_set_read_write() {
    board_init();

    uint8_t pin_modes[NUM_PINS];
    pin_config_t pin_config;
    for (int i=0; i<NUM_PINS; i++) pin_config.pin_modes[i] = MODE_DIGITAL_OUTPUT;

    board_set_pins(&pin_config);

    printf("%d\n", pin_config.pin_modes[35]);
    printf("%d\n", MODE_DIGITAL_OUTPUT);
    printf("%d\n", board.pin_modes[35] == MODE_DIGITAL_OUTPUT);

    //double vals_in[NUM_PINS];
    //for (int i=0; i<NUM_PINS; i++) vals_in[i] = i;
    //board_write(&vals_in);

    //for (int i=0; i<NUM_PINS; i++) pin_config.pin_modes[i] = MODE_DIGITAL_INPUT;
    //board_set_pins(&pin_config);

    //printf("%f\n", pin_values[35]);

    //double vals_out[NUM_PINS];
    //board_read(&vals_out);

    //for (int i=0; i<NUM_PINS; i++) TEST_ASSERT_FLOAT_WITHIN(1e-5, i, vals_out[i]);
}
