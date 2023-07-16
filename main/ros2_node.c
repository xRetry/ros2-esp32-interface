#include <discovery.h>
#include <init.h>
#include <rcl/publisher.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <timing.h>
#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>
//#include <unistd.h>

#include "modes.h"
#include "ros2_node.h"
#include "sdkconfig.h"
#include "stdbool.h"
//#include "tracetools/tracetools.h"


#define OK_OR_CLEANUP(fn) {\
    rcl_ret_t err = fn;\
    if ((err != RCL_RET_OK)) {\
        printf("Failed status on line %d: %d.\n",\
            __LINE__, (int) err\
        );\
        board.node_error = err;\
        goto cleanup;\
    }\
}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rcl_ret_t check(void *arg, rcl_ret_t ret) {
    RCL_CHECK_ARGUMENT_FOR_NULL(arg, ret);
    return RCL_RET_OK;
}

pin_values_t pub_msg;
pin_values_t sub_msg;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_service_t service;
rclc_support_t support;

void handle_write_pins(const void *msgin) {
    const pin_values_t *msg = (const pin_values_t*) msgin;

    board_write(&msg->values);
}

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer != NULL) {
        board_read(&pub_msg.values);
        rcl_ret_t err = rcl_publish(&publisher, &pub_msg, NULL);
        board.node_error = err;
    }
}

void handle_set_config(const void *msg_req, void *msg_rsp) {
    printf(">>> Changing config\n");
    // Cast messages to expected types
    set_config_req_t * request = (set_config_req_t *) msg_req;
    set_config_rsp_t * response = (set_config_rsp_t *) msg_rsp;

    pthread_rwlock_wrlock(&board.lock);

    if (!request->read_only) { 
        printf(">>> Changing pins\n");
        board_set_pins(request->pin_modes);
    }

    pthread_rwlock_unlock(&board.lock);

    printf(">>> Generating config response\n");
    for (int i=0; i<NUM_PINS; i++) {
        response->pin_modes[i] = board.pin_modes[i];
        response->pin_errors[i] = board.pin_errors[i];
    }
    for (int i=0; i<NUM_PINS; i++) {
        printf("pin_nr: %d\nmode: %d\nerr: %d (%s)\n", 
               i, response->pin_modes[i], response->pin_errors[i], esp_err_to_name(response->pin_errors[i]));
    }
}

bool node_transport_init() {
    printf(">>> Init wifi\n");
    board.transport_error = uros_network_interface_initialize();
    return board.transport_error == ESP_OK;
}

bool node_init() {
    printf(">>> Init node\n");

    bool is_init_options_init = false;
    bool is_exec_init = false;
    bool is_pub_init = false;
    bool is_sub_init = false;
    bool is_service_init = false;
    bool is_node_init = false;
    bool is_supp_init = false;
    bool is_timer_init = false;

    printf(">>> Init options\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	OK_OR_CLEANUP(rcl_init_options_init(&init_options, allocator));
    is_init_options_init = true;

    printf(">>> Init RMW\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	OK_OR_CLEANUP(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    
    // --- Wait for agent ---
	while (rmw_uros_ping_agent_options(2000, 1, rmw_options) != RMW_RET_OK) {
        printf(">>> Waiting for agent...\n");
    }

    printf(">>> Init support\n");
	OK_OR_CLEANUP(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    is_supp_init = true;

    // --- Create node ---

    printf(">>> Init node\n");
    OK_OR_CLEANUP(rclc_node_init_default(&node, CONFIG_MICRO_ROS_NODE_NAME, CONFIG_MICRO_ROS_NAMESPACE, &support));
    is_node_init = true;

    // --- Create executor ---
    //
    printf(">>> Init executor\n");
    executor = rclc_executor_get_zero_initialized_executor();
    OK_OR_CLEANUP(rclc_executor_init(&executor, &support.context, 3, &allocator));
    is_exec_init = true;

    // --- Create publisher ---

    printf(">>> Init publisher\n");
    OK_OR_CLEANUP(rclc_publisher_init_best_effort(
        &publisher,
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_messages, msg, PinValues),
        CONFIG_MICRO_ROS_PUBLISHER_NAME
    ));
    is_pub_init = true;

    // --- Create subscriber ---
    
    printf(">>> Init subscriber\n");
    subscriber = rcl_get_zero_initialized_subscription();
    OK_OR_CLEANUP(rclc_subscription_init_best_effort(
        &subscriber, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_messages, msg, PinValues),
        CONFIG_MICRO_ROS_SUBSCRIBER_NAME
    ));
    is_sub_init = true;

     // Add subscriber to executor
     OK_OR_CLEANUP(rclc_executor_add_subscription(
         &executor,
         &subscriber,
         &sub_msg,
         handle_write_pins,
         ON_NEW_DATA
     ));

    // --- Create service ---
    
    printf(">>> Init service\n");
    rcl_service_t service;
    OK_OR_CLEANUP(rclc_service_init_default(
        &service, 
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_messages, srv, SetConfig), 
        CONFIG_MICRO_ROS_SERVICE_NAME
    ));
    is_service_init = true;
    
    set_config_req_t set_pin_req;
    set_config_rsp_t set_pin_rsp;

    // Add service to executor
    OK_OR_CLEANUP(rclc_executor_add_service(
        &executor, 
        &service, 
        &set_pin_req, 
        &set_pin_rsp, 
        &handle_set_config
    ));

    // --- Create timer ---

    printf(">>> Init timer\n");
    rcl_timer_t timer  = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = CONFIG_MICRO_ROS_REFRESH_RATE_MS;
    OK_OR_CLEANUP(rclc_timer_init_default(
        &timer, 
        &support, 
        RCL_MS_TO_NS(timer_timeout),
        handle_read_pins
    ));
    is_timer_init = true;
    // Add timer and subscriber to executor.
    OK_OR_CLEANUP(rclc_executor_add_timer(&executor, &timer));

    // --- Run node ---
    
    printf(">>> Run node\n");
    rmw_ret_t status = RMW_RET_OK;
    while (1) {
        EXECUTE_EVERY_N_MS(2000, status = rmw_uros_ping_agent_options(100, 1, rmw_options););
        if (status != RMW_RET_OK) goto cleanup;
        rclc_executor_spin_some(&executor, 100);
    }

cleanup:
    printf(">>> Cleanup node\n");
    rcl_ret_t err = RCL_RET_OK;

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    if (is_init_options_init) err += rcl_init_options_fini(&init_options);
    if (is_sub_init) err += rcl_subscription_fini(&subscriber, &node);
    if (is_pub_init) err += rcl_publisher_fini(&publisher, &node);
    if (is_service_init) err += rcl_service_fini(&service, &node);
    if (is_timer_init) err += rcl_timer_fini(&timer);
    if (is_exec_init) err += rclc_executor_fini(&executor);
    if (is_node_init) err += rcl_node_fini(&node);
    if (is_supp_init) err += rclc_support_fini(&support);
    return err == RCL_RET_OK;
}



