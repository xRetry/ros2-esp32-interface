#include <init.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>
#include <unistd.h>

#include "ros2_node.h"
#include <std_msgs/msg/int32.h>

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


pin_values_t pub_msg;
pin_values_t sub_msg;
rcl_publisher_t publisher;


void handle_write_pins(const void *msgin) {
    printf(">>> Write pins\n");
    const pin_values_t *msg = (const pin_values_t*) msgin;

    board_write(&msg->values);
}

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time) {
    printf(">>> Read pins\n");

    (void)last_call_time;

    if (timer != NULL) {
        board_read(&pub_msg.values);
        rcl_ret_t err = rcl_publish(&publisher, &pub_msg, NULL);
        //int_msg.data = 10;
        //rcl_ret_t err = rcl_publish(&board.publisher, &int_msg, NULL);
        board.node_error = err;
    }
}

void handle_set_config(const void *msg_req, void *msg_rsp) {
    printf(">>> Changing config\n");
    // Cast messages to expected types
    set_config_req_t * request = (set_config_req_t *) msg_req;
    set_config_rsp_t * response = (set_config_rsp_t *) msg_rsp;


    bool change_pins = (bool) request->change_pins;
    bool change_node = (bool) request->change_node;
    bool change_transport = (bool) request->change_transport;

    pthread_rwlock_wrlock(&board.lock);

    if (change_pins) {
        printf(">>> Changing pins\n");
        pin_config_t *pin_config =  &request->new_pin_config;
        board_set_pins(pin_config);
    }

    if (change_node) {
        printf(">>> Changing node\n");
        node_config_t *node_config = &request->new_node_config;
        board.refresh_rate_ms = node_config->refresh_rate_ms;
        board.node_name = node_config->node_name.data;
        board.publisher_name = node_config->publisher_name.data;
        board.subscriber_name = node_config->subscriber_name.data;
        board.service_name = node_config->service_name.data;
    }

    if (change_transport) {
        printf(">>> Changing transport\n");
        transport_config_t *transport_config = &request->new_transport_config;
        board.use_wifi = transport_config->use_wifi;
        board.agent_ip = transport_config->agent_ip.data;
        board.agent_port = transport_config->agent_port.data;
        board.wifi_ssid = transport_config->wifi_ssid.data;
        board.wifi_pw = transport_config->wifi_pw.data;
    }

    if (change_node || change_transport) {
        printf(">>> Node shutdown\n");

        // NOTE: This will stop the node and therefore current thread.
        // Afterwards the node will restart automatically.
        // A response will never be sended.
        rcl_ret_t err = node_shutdown();

        printf(">>> Node shutdown failed\n");
        board.node_error = err;
    }

    pthread_rwlock_unlock(&board.lock);

    printf(">>> Generating config response\n");
    for (int i=0; i<NUM_PINS; i++) {
        response->active_pin_config.pin_modes[i] = board.pin_modes[i];
        response->pin_error[i] = board.pin_errors[i];
    }
    response->active_node_config.refresh_rate_ms = board.refresh_rate_ms;
    response->active_node_config.node_name.data = board.node_name;
    response->active_node_config.publisher_name.data = board.publisher_name;
    response->active_node_config.subscriber_name.data = board.subscriber_name;
    response->active_node_config.service_name.data = board.service_name;
    response->node_error = board.node_error;

    response->active_transport_config.use_wifi = board.use_wifi;
    response->active_transport_config.agent_ip.data = board.agent_ip;
    response->active_transport_config.agent_port.data = board.agent_port;
    response->active_transport_config.wifi_ssid.data = board.wifi_ssid;
    response->active_transport_config.wifi_pw.data = board.wifi_pw;
    response->transport_error = board.transport_error;
}

bool node_transport_init() {
    if (board.use_wifi) {
        printf(">>> Init wifi\n");
        board.transport_error = uros_network_interface_initialize();
    }
    return board.transport_error == ESP_OK;
}

bool node_ping_agent() {
    printf(">>> Ping agent\n");
    return true;
    return rmw_uros_ping_agent(1000, 3) == RMW_RET_OK;
}

bool node_init() {
    printf(">>> Init node\n");

    usleep(1e6);

    printf(">>> Init options\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	OK_OR_CLEANUP(rcl_init_options_init(&init_options, allocator));

    printf(">>> Init RMW\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	OK_OR_CLEANUP(rmw_uros_options_set_udp_address(board.agent_ip, board.agent_port, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
    
    printf(">>> Init support\n");
	OK_OR_CLEANUP(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


    // --- Create node ---

    printf(">>> Init node\n");
    rcl_node_t node;
    OK_OR_CLEANUP(rclc_node_init_default(&node, board.node_name, "", &support));

    // --- Create executor ---
    //
    printf(">>> Init executor\n");
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rcl_context_t context;
    OK_OR_CLEANUP(rclc_executor_init(&executor, &context, 1, &allocator));
    //unsigned int rcl_wait_timeout = board.refresh_rate_ms; // in ms
    //unsigned int rcl_wait_timeout = 1000;
    //RCCHECK(rclc_executor_set_timeout(&board.executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // --- Create publisher ---

    printf(">>> Init publisher\n");
    OK_OR_CLEANUP(rclc_publisher_init_best_effort(
        &publisher,
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.publisher_name
    ));

    // --- Create subscriber ---
    
    printf(">>> Init subscriber\n");
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    OK_OR_CLEANUP(rclc_subscription_init_best_effort(
        &subscriber, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.subscriber_name
    ));

     // Add subscriber to executor
     OK_OR_CLEANUP(rclc_executor_add_subscription(
         &executor,
         &subscriber,
         &sub_msg,
         handle_write_pins,
         ON_NEW_DATA
     ));

    // --- Create service ---
    
    //printf(">>> Init service\n");
    //RCCHECK(rclc_service_init_default(
    //    &board.service, 
    //    &board.node,
    //    ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_interfaces, srv, SetConfig), 
    //    "test"//board.service_name
    //));
    //
    //set_config_req_t set_pin_req;
    //set_config_rsp_t set_pin_rsp;

    //// Add service to executor
    //RCCHECK(rclc_executor_add_service(
    //    &board.executor, 
    //    &board.service, 
    //    &set_pin_req, 
    //    &set_pin_rsp, 
    //    &handle_set_config
    //));

    // --- Create timer ---

    printf(">>> Init timer\n");
    rcl_timer_t timer  = rcl_get_zero_initialized_timer();
    //const unsigned int timer_timeout = board.refresh_rate_ms;
    const unsigned int timer_timeout = 500;
    OK_OR_CLEANUP(rclc_timer_init_default(
        &timer, 
        &support, 
        RCL_MS_TO_NS(timer_timeout),
        handle_read_pins
    ));
    // Add timer and subscriber to executor.
    OK_OR_CLEANUP(rclc_executor_add_timer(&executor, &timer));

    // --- Run node ---
    
    printf(">>> Run node\n");
    //rmw_ret_t status = RMW_RET_OK;
    //while (1) {
    //    EXECUTE_EVERY_N_MS(1000, status = rmw_uros_ping_agent(100, 1););
    //    if (status != RMW_RET_OK) goto cleanup;
    //    rcl_ret_t err = rclc_executor_spin_some(&executor, 100);
    //    printf("error: %d\n", err);
    //    //printf("%d\n", rcl_context_is_valid(board.executor.context));
    //    break;

    //    usleep(500);
    //    //printf("after sleep: %d\n", rcl_context_is_valid(board.executor.context));
    //}
    rclc_executor_spin(&executor);

cleanup:
    // TODO: Only cleanup initialized parts
    {
        printf(">>> Cleanup node\n");
        rcl_ret_t err = RCL_RET_OK;
        err += rclc_executor_fini(&executor);
        err += rcl_subscription_fini(&subscriber, &node);
        err += rcl_publisher_fini(&publisher, &node);
        //RCCHECK(rcl_service_fini(&service, &node));
        err += rcl_node_fini(&node);
        err += rclc_support_fini(&support);
        return err == RCL_RET_OK;
    }
}

