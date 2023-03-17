#include <executor_handle.h>
#include <rcl/allocator.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
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
#include <rmw_microros/rmw_microros.h>

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

#define RMW_UXRCE_TRANSPORT_SERIAL 1
#define RMW_UXRCE_TRANSPORT_UDP 1

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

void connect_to_agent(rcl_init_options_t *init_options, rclc_support_t *support, rcl_allocator_t *allocator) {
    printf("connect_to_agent\n");

    bool try_udp = true;
    esp_err_t err = ESP_FAIL;
    while (err != ESP_OK) {
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(init_options);

        if (try_udp) {
            printf("udp\n");
            RCCHECK(rmw_uros_options_set_udp_address(
                "10.0.0.39",
                "8888", 
                rmw_options
            ));
        } else {
            rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(init_options);

            printf("serial\n");
            printf("%d\n", NULL == rmw_options);
            #if defined (RMW_UXRCE_TRANSPORT_SERIAL)
            printf("serial defined\n");
            #else
            printf("serial not defined\n");
            #endif
            const char *dev = "/dev/ttyUSB0";
            printf("%d\n", dev != NULL && strlen(dev) <= MAX_SERIAL_DEVICE);
            //RCCHECK(rmw_uros_options_set_serial_device(dev, rmw_options));
            //printf("", ,rmw_options->impl->transport_params.serial_device);
            printf("%d", rmw_options->impl == NULL);
            //snprintf(rmw_options->impl->transport_params.serial_device, MAX_SERIAL_DEVICE, "%s", dev);
            RCCHECK(rmw_uros_options_set_serial_device(dev, rmw_options));
        }

        printf("support\n");
       
        err = rclc_support_init_with_options(
            support, 
            0, 
            NULL, 
            init_options,
            allocator
        );
        
        try_udp = !try_udp;
    }
}

esp_err_t connect(rcl_allocator_t *allocator, bool wifi) {
    // Init RCL options and context
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    *allocator = rcl_get_default_allocator();
    printf("init options\n");
    RCCHECK(rcl_init_options_init(&init_options, *allocator));

    printf("init rmw options\n");
    // Take RMW options from RCL options
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    if (wifi) {
        printf("try wifi\n");
        // TCP/UDP case: Set RMW IP parameters
        rmw_uros_options_set_udp_address("10.0.0.38", "8888", rmw_options);
    } else {
        printf("try serial\n");
        // Serial case: Set RMW serial device parameters
        rmw_uros_options_set_serial_device("/dev/ttyUSB0", rmw_options);
    }

    //printf("set client key\n");
    // Set RMW client key
    //rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);

    // Init RCL
    printf("init rcl\n");
    esp_err_t err = rcl_init(0, NULL, &init_options, &context);
    if (err != ESP_OK) {
        printf("error %d\n", err);
        //RCCHECK(rcl_shutdown(&context));
        RCCHECK(rcl_init_options_fini(&init_options));
    }
    return err;

    // ... micro-ROS code ...
}

void init_node(void *arg) {
    printf("Enter init_node\n");
    RCCHECK(board_init());

    //rcl_allocator_t allocator;
    //rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rclc_support_t support;
    //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    //RCCHECK(rcl_init_options_init(&init_options, allocator));
    //RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));
    //printf("%d\n", init_options.impl->rmw_init_options == NULL);
    //printf("%d\n", init_options.impl->allocator == NULL);
    //connect_to_agent(&init_options, &support, &allocator);
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))
    
    //esp_err_t err = ESP_FAIL;
    //bool try_wifi = true;
    //while (err != ESP_OK) {
    //    err = connect(&allocator, try_wifi);
    //    try_wifi = !try_wifi;
    //    vTaskDelay(2000/ portTICK_PERIOD_MS);
    //}

    printf("init success\n");

    // --- Create node ---

    //rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_t node; 
    RCCHECK(rclc_node_init_default(&node, "esp32_interface", "", &support));

    // --- Create executor ---
    printf("executor\n");

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // --- Create publisher ---

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        "esp32_read_pins"
    ));


    // --- Create subscriber ---
    
    //rcl_subscription_t subscriber;
    //RCCHECK(rclc_subscription_init_best_effort(
    //    &subscriber, 
    //    &node, 
    //    ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
    //    "esp32_write_pins"
    //));

    // Add subscriber to executor
    //RCCHECK(rclc_executor_add_subscription(
    //    &executor,
    //    &subscriber,
    //    &recv_msg,
    //    &handle_write_pins,
    //    ON_NEW_DATA
    //));

    // --- Create service ---
    
    // Initialize server with default configuration
    //rcl_service_t service;
    //RCCHECK(rclc_service_init_default(
    //    &service, 
    //    &node,
    //    ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_interfaces, srv, SetPin), 
    //    "/esp32_set_pin"
    //));

    //
    //set_pin_req_t set_pin_req;
    //set_pin_rsp_t set_pin_rsp;

    //RCCHECK(rclc_executor_add_service(
    //        &executor, 
    //        &service, 
    //        &set_pin_req, 
    //        &set_pin_rsp, 
    //        &handle_set_pin
    //));

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

    //RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    //RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);

}

void test_board() {
    board_init();

    set_pin_req_t req;
    req.new_config.pin_mode = MODE_DIGITAL_OUTPUT;
    req.new_config.pin_nr = 13;
    board_set_pin(&req);

    req.new_config.pin_mode = MODE_DIGITAL_INPUT;
    req.new_config.pin_nr = 14;
    board_set_pin(&req);

    req.new_config.pin_mode = MODE_ANALOG_INPUT;
    req.new_config.pin_nr = 26;
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
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    printf("init network");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(init_node, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    //test_board();
}
