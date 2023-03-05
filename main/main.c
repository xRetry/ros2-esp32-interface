#include <executor_handle.h>
#include <unistd.h>

#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "board.h"

#include <uros_network_interfaces.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

rcl_publisher_t publisher;
pin_values_t recv_msg;


#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__,            \
             (int)temp_rc);                                                    \
      vTaskDelete(NULL);                                                       \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__,          \
             (int)temp_rc);                                                    \
    }                                                                          \
  }

void handle_write_pins(const void *msgin) {
    const pin_values_t *msg = (const pin_values_t*) msgin;

    board_write(&msg->vals);
}

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer != NULL) {
        pin_values_t msg;
        board_read(&msg.vals);
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

void handle_set_pin(const void *msg_req, void *msg_rsp) {
    // Cast messages to expected types
    set_pin_req_t * request = (set_pin_req_t *) msg_req;
    set_pin_rsp_t * response = (set_pin_rsp_t *) msg_rsp;

    esp_err_t err = board_set_pin(request);

    response->is_ok = err == ESP_OK;
}

void init_node(void *arg) {
    printf("Enter init_node\n");
    esp_err_t err = board_init();

    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    printf("rmw\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // --- Create support ---
    printf("support\n");
   
    rclc_support_t support;
    RCCHECK(rclc_support_init_with_options(
        &support, 
        0, 
        NULL, 
        &init_options,
        &allocator
    ));

    // --- Create node ---

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_interface", "", &support));

    // --- Create executor ---
    printf("executor\n");

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // --- Create publisher ---

    RCCHECK(rclc_publisher_init_default(
        &publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        "esp32_read_pins"
    ));


    // --- Create subscriber ---
    
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        "esp32_write_pins"
    ));

    // Add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &recv_msg,
        &handle_write_pins,
        ON_NEW_DATA
    ));

    // --- Create service ---
    
    // Get message type support
    const rosidl_service_type_support_t * type_support =
        ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_interfaces, srv, SetPin);

    // Initialize server with default configuration
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(
        &service, 
        &node,
        type_support, 
        "/esp32_set_pin"
    ));

    // --- Create timer ---

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer, 
        &support, 
        RCL_MS_TO_NS(timer_timeout),
        handle_read_pins
    ));
    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_timer(&executor, &timer));



    // --- Run endless loop ---

    printf("Enter init_node endless loop\n");
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000));
        usleep(100000);
    }

    // --- Free resources ---

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);

}

void test_board() {
    board_init();

    set_pin_req_t req;
    req.new_config.type = DIGITAL;
    req.new_config.pin_nr = 13;
    req.new_config.direction = OUTPUT;
    board_set_pin(&req);

    req.new_config.type = DIGITAL;
    req.new_config.pin_nr = 14;
    req.new_config.direction = INPUT;
    board_set_pin(&req);

    req.new_config.type = ANALOG;
    req.new_config.pin_nr = 26;
    req.new_config.direction = INPUT;
    board_set_pin(&req);

    double vals[35];
    double vals_out[35];
    while (1) {
        vals[13] = 1.;
        board_write(&vals);
        board_read(&vals_out);
        printf("read: %f\n", vals_out[14]);
        printf("read2: %f\n", vals_out[26]);
        vTaskDelay(1000/ portTICK_PERIOD_MS);

        vals[13] = 0.;
        board_write(&vals);
        board_read(&vals_out);
        printf("read: %f\n", vals_out[14]);
        printf("read2: %f\n", vals_out[26]);
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

void app_main(void) {
//#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
//    printf("init network");
//    ESP_ERROR_CHECK(uros_network_interface_initialize());
//#endif
//
//    xTaskCreate(init_node, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
//        CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    test_board();
}
