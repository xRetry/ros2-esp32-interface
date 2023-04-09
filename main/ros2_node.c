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
//#include "sdkconfig.h"

#define RCCHECK(fn) {\
    rcl_ret_t err = fn;\
    if ((err != RCL_RET_OK)) {\
        printf("Failed status on line %d: %d.\n",\
            __LINE__, (int) err\
        );\
    }\
    return err;\
}

pin_values_t recv_msg;

rcl_ret_t node_shutdown() {
    RCCHECK(rcl_subscription_fini(&board.subscriber, &board.node));
    RCCHECK(rcl_publisher_fini(&board.publisher, &board.node));
    RCCHECK(rcl_service_fini(&board.service, &board.node));
    RCCHECK(rcl_node_fini(&board.node));
}

void handle_write_pins(const void *msgin) {
    const pin_values_t *msg = (const pin_values_t*) msgin;

    board_write((double (*)[NUM_PINS]) &msg->values);
}

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer != NULL) {
        pin_values_t msg;
        board_read((double (*)[NUM_PINS]) &msg.values);
        rcl_ret_t err = rcl_publish(&board.publisher, &msg, NULL);
        board.node_error = err;
    }
}

void handle_set_config(const void *msg_req, void *msg_rsp) {
    // Cast messages to expected types
    set_config_req_t * request = (set_config_req_t *) msg_req;
    set_config_rsp_t * response = (set_config_rsp_t *) msg_rsp;


    bool change_pins = (bool) request->change_pins;
    bool change_node = (bool) request->change_node;
    bool change_transport = (bool) request->change_transport;

    pthread_rwlock_wrlock(&board.lock);

    if (change_pins) {
        pin_config_t *pin_config =  &request->new_pin_config;
        board_set_pins(pin_config);
    }

    if (change_node) {
        node_config_t *node_config = &request->new_node_config;
        board.refresh_rate_ms = node_config->refresh_rate_ms;
        board.node_name = node_config->node_name.data;
        board.publisher_name = node_config->publisher_name.data;
        board.subscriber_name = node_config->subscriber_name.data;
        board.service_name = node_config->service_name.data;
    }

    if (change_transport) {
        transport_config_t *transport_config = &request->new_transport_config;
        board.use_wifi = transport_config->use_wifi;
        board.agent_ip = transport_config->agent_ip.data;
        board.agent_port = transport_config->agent_port.data;
        board.wifi_ssid = transport_config->wifi_ssid.data;
        board.wifi_pw = transport_config->wifi_pw.data;
    }

    if (change_node || change_transport) {
        // TODO: Retry shutdown if fail
        rcl_ret_t err = node_shutdown();
        // TODO: Swtich to fallback if error
        board.node_error = err;
    }

    pthread_rwlock_unlock(&board.lock);

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

rcl_ret_t node_try_init() {
    printf("Enter init_node\n");


    if (board.use_wifi) {
        printf("init network\n");
        esp_err_t err = uros_network_interface_initialize();
        printf("%d\n", err);
    }

    printf("cp\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

    printf("cp\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(board.agent_ip, board.agent_port, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    printf("init success\n");

    // --- Create node ---

    RCCHECK(rclc_node_init_default(&board.node, board.node_name, "", &support));

    // --- Create executor ---
    printf("executor\n");

    board.executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&board.executor, &support.context, 3, &allocator));
    unsigned int rcl_wait_timeout = board.refresh_rate_ms; // in ms
    RCCHECK(rclc_executor_set_timeout(&board.executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // --- Create publisher ---

    RCCHECK(rclc_publisher_init_best_effort(
        &board.publisher, 
        &board.node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.publisher_name
    ));


    // --- Create subscriber ---
    
    RCCHECK(rclc_subscription_init_best_effort(
        &board.subscriber, 
        &board.node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.subscriber_name
    ));

    // Add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &board.executor,
        &board.subscriber,
        &recv_msg,
        &handle_write_pins,
        ON_NEW_DATA
    ));

    // --- Create service ---
    
    // Initialize server with default configuration
    RCCHECK(rclc_service_init_default(
        &board.service, 
        &board.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_interfaces, srv, SetConfig), 
        board.service_name
    ));
    
    set_config_req_t set_pin_req;
    set_config_rsp_t set_pin_rsp;

    RCCHECK(rclc_executor_add_service(
        &board.executor, 
        &board.service, 
        &set_pin_req, 
        &set_pin_rsp, 
        &handle_set_config
    ));

    // --- Create timer ---

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = board.refresh_rate_ms;
    RCCHECK(rclc_timer_init_default(
        &timer, 
        &support, 
        RCL_MS_TO_NS(timer_timeout),
        handle_read_pins
    ));
    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_timer(&board.executor, &timer));
}

void node_run() {
    printf("Enter init_node endless loop\n");
    // --- Run endless loop ---
    while (1) {
        rclc_executor_spin_some(&board.executor, RCL_MS_TO_NS(10000));
        usleep(100000);
    }
}

void node_init() {
    while(1) {
        rcl_ret_t err = node_try_init();
        board.node_error = err;
        if (err != RCL_RET_OK) {
            // TODO: Don't reset pins
            board_init(); 
            err = node_try_init();
        }
        node_run();
    }
}
