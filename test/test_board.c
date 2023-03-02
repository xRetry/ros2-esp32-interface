#include "unity.h"

#define CONFIG_MBEDTLS_HAVE_TIME 1
#define MBEDTLS_HAVE_TIME 1

#include "board.h"
#include "driver/mock_gpio.h"


void setUp(void) {

}

void test_init_board(void) {
    esp_err_t err = board_init();
    TEST_ASSERT_EQUAL_INT(ESP_OK, err);
}
