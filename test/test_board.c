#include "unity.h"
#include "board.h"
//#include "driver/mock_gpio.h"
#include "mock_gpio.h"

//TEST_FILE("driver/gpio.h");

void setUp(void) {

}

void test_init_board(void) {
    esp_err_t err = board_init();
    TEST_ASSERT_EQUAL_INT(ESP_OK, err);
}
