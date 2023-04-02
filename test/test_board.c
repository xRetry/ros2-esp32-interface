#define CONFIG_IDF_TARGET_ESP32 1
#define CONFIG_MICRO_ROS_AGENT_IP "127.0.0.1"
#define CONFIG_MICRO_ROS_AGENT_PORT "8888"
#include "unity.h"
#include "board.h"
//#include "mock_driver/gpio.h"


void setUp(void) {

}

void test_init_board(void) {
    //board_init();
    //TEST_ASSERT_EQUAL_INT(ESP_OK, err);
}
