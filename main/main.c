#include <executor_handle.h>
#include <ros2_esp32_interfaces/srv/detail/set_pin__functions.h>
#include <unistd.h>

#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <std_msgs/msg/int32.h>
#include <ros2_esp32_interfaces/msg/pin_values.h>
#include <ros2_esp32_interfaces/srv/set_pin.h>

#include "hardware_bindings.h"

typedef ros2_esp32_interfaces__msg__PinValues pin_values_t;

//typedef ros2_esp32_interfaces__srv__SetPin_Request set_pin_req_t;

typedef struct set_pin_req_t {
    int pin_nr;
    pin_mode_t mode;    
    pin_type_t type;
} set_pin_req_t;

typedef struct  set_pin_rsp_t {
    bool is_ok;
} set_pin_rsp_t;

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

    pthread_rwlock_rdlock(&board.lock);
    for (int i=0; i<NUM_PINS; i++) {
        if (board.mode[i] == OUTPUT) {
            double val = msg->vals[i];
            esp_err_t err = gpio_set_level(board.pins[i], (uint32_t) val);
        }
    }
    pthread_rwlock_unlock(&board.lock);
}

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        pthread_rwlock_rdlock(&board.lock);

        pin_values_t msg;
        for (int i=0; i<NUM_PINS; i++) {
            if (board.mode[i] == INPUT) {
                int val = gpio_get_level(board.pins[i]);
                msg.vals[i] = val;
            }
        }

        pthread_rwlock_unlock(&board.lock);
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

void handle_set_pin(const void *msg_req, void *msg_rsp) {
    // Cast messages to expected types
    set_pin_req_t * request = (set_pin_req_t *) msg_req;
    set_pin_rsp_t * response = (set_pin_rsp_t *) msg_rsp;

    // Handle wrong pin_nr
    // TODO: Use built-in macros
    if (request->pin_nr < 0 && request->pin_nr >= NUM_PINS) {
        response->is_ok = false;
        return;
    }

    // Change pin
    pthread_rwlock_wrlock(&board.lock);

    bool is_ok = false;
    switch (request->mode) {
    case DISABLED: 
        is_ok = set_pin_disabled(request->pin_nr);
        break;
    case INPUT:
        switch (request->type) {
        case DIGITAL:
            is_ok = set_pin_digital_input(request->pin_nr);
            break;
        case ANALOG:
            is_ok = set_pin_analog_input(request->pin_nr);
            break;
        }
        break;
    case OUTPUT:
        switch (request->type) {
        case DIGITAL:
            is_ok = set_pin_digital_output(request->pin_nr);
            break;
        case ANALOG:
            is_ok = set_pin_analog_output(request->pin_nr);
            break;
        }
        break;
    }

    if (is_ok) {
        board.mode[request->pin_nr] = request->mode;
        board.type[request->pin_nr] = request->type;
    }
    pthread_rwlock_unlock(&board.lock);

    response->is_ok = is_ok;
}

void init_node(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // --- Create support ---
   
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

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // --- Create publisher ---

    RCCHECK(rclc_publisher_init_default(
        &publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32_read_pins"
    ));


    // --- Create subscriber ---
    
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
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

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // --- Free resources ---

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);

}

void app_main(void) {
    xTaskCreate(init_node, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
