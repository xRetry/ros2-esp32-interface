//#include "freertos/portmacro.h"
//#include <executor_handle.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ros2_node.h"

//void test_board() {
//    board_init();
//
//    set_pin_req_t req;
//    req.new_config.pin_mode = MODE_DIGITAL_INPUT;
//    req.new_config.pin_nr = 26;
//    board_set_pin(&req);
//
//    req.new_config.pin_mode = MODE_DIGITAL_OUTPUT;
//    req.new_config.pin_nr = 14;
//    board_set_pin(&req);
//
//    req.new_config.pin_mode = MODE_ANALOG_INPUT;
//    req.new_config.pin_nr = 33;
//    board_set_pin(&req);
//
//    req.new_config.pin_mode = MODE_ANALOG_OUTPUT;
//    req.new_config.pin_nr = 25;
//    board_set_pin(&req);
//
//    double vals[35];
//    double vals_out[35];
//    while (1) {
//        vals[14] = 1.;
//        vals[25] = 200.;
//        board_write(&vals);
//        board_read(&vals_out);
//        printf("read digital: %f\n", vals_out[26]);
//        printf("read2 analog: %f\n", vals_out[33]);
//        vTaskDelay(1000/ portTICK_PERIOD_MS);
//
//        vals[14] = 0.;
//        vals[25] = 50.;
//        board_write(&vals);
//        board_read(&vals_out);
//        printf("read digital: %f\n", vals_out[26]);
//        printf("read analog: %f\n", vals_out[33]);
//        vTaskDelay(1000/ portTICK_PERIOD_MS);
//    }
//}
//
//

enum states {
    STATE_TRANSPORT_INIT,
    STATE_NODE_RUN,
} state;

void run_state_machine() {
    board_init(); 

    while (1) {
        switch (state) {
            case STATE_TRANSPORT_INIT:
                state = node_transport_init() ? STATE_NODE_RUN : STATE_TRANSPORT_INIT;
                break;
            case STATE_NODE_RUN:
                node_init();
                break;
            default:
                state = STATE_TRANSPORT_INIT;
                break;
        }

    }

    vTaskDelete(NULL);
}

void app_main(void) {
    xTaskCreate(run_state_machine, "uros_task", 16000, NULL, 5, NULL);
    //test_board();
}
