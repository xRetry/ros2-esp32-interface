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

#include "modes.h"
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

bool is_exec_init = false;
bool is_pub_init = false;
bool is_sub_init = false;
bool is_service_init = false;
bool is_node_init = false;
bool is_supp_init = false;

rcl_ret_t node_shutdown() {
    printf(">>> Cleanup node\n");
    rcl_ret_t err = RCL_RET_OK;

    if (is_exec_init) err += rclc_executor_fini(&executor);
    if (is_sub_init) err += rcl_subscription_fini(&subscriber, &node);
    if (is_pub_init) err += rcl_publisher_fini(&publisher, &node);
    if (is_service_init) err += rcl_service_fini(&service, &node);
    if (is_node_init) err += rcl_node_fini(&node);
    if (is_supp_init) err += rclc_support_fini(&support);
    return err == RCL_RET_OK;
}

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
        board.node_error = err;
    }
}

void handle_set_config(const void *msg_req, void *msg_rsp) {
    printf(">>> Changing config\n");
    // Cast messages to expected types
    set_config_req_t * request = (set_config_req_t *) msg_req;
    set_config_rsp_t * response = (set_config_rsp_t *) msg_rsp;

    bool change_pins = request->change_pins;
    bool change_node = request->change_node;
    bool change_transport = request->change_transport;
    printf("cp: %d, cn: %d, ct: %d\n", change_pins, change_node, change_transport);
    for (int i=0; i<NUM_PINS; i++) {
        printf("pin: %d, mode: %d\n", i, request->pin_modes[i]);
    }

    pthread_rwlock_wrlock(&board.lock);

    if (change_pins) {
        printf(">>> Changing pins\n");
        board_set_pins(request->pin_modes);
    }

    if (change_node) {
        printf(">>> Changing node\n");
    //    board.refresh_rate_ms = request->refresh_rate_ms;
    //    board.node_name = request->node_name.data;
    //    board.publisher_name = request->publisher_name.data;
    //    board.subscriber_name = request->subscriber_name.data;
    //    board.service_name = request->service_name.data;
    }

    if (change_transport) {
        printf(">>> Changing transport\n");
    //    board.use_wifi = request->use_wifi;
    //    printf("%d\n", board.use_wifi);
    //    free(board.agent_ip);
    //    printf("%s\n", request->agent_ip.data);
    //    board.agent_ip = strdup(request>agent_ip.data);
    //    free(board.agent_port);
    //    board.agent_port = strdup(request->agent_port.data);
    //    free(board.wifi_ssid);
    //    board.wifi_ssid = strdup(request->wifi_ssid.data);
    //    free(board.wifi_pw);
    //    board.wifi_pw = strdup(request->wifi_pw.data);
    }

    if (change_node || change_transport) {
        printf(">>> Node shutdown\n");

    //    // NOTE: This will stop the node and therefore current thread.
    //    // Afterwards the node will restart automatically.
    //    // A response will never be sended.
    //    rcl_ret_t err = node_shutdown();

    //    printf(">>> Node shutdown failed\n");
    //    board.node_error = err;
    //    pthread_rwlock_unlock(&board.lock);
    //    return;
    }

    pthread_rwlock_unlock(&board.lock);

    printf(">>> Generating config response\n");
    for (int i=0; i<NUM_PINS; i++) {
        response->pin_modes[i] = board.pin_modes[i];
        response->pin_error[i] = board.pin_errors[i];
    }
    for (int i=0; i<NUM_PINS; i++) {
        printf("pin_nr: %d\nmode: %d\nerr: %d\n", 
               i, response->pin_modes[i], response->pin_error[i]);
    }
    //response->refresh_rate_ms = board.refresh_rate_ms;
    //response->node_name.data = board.node_name;
    //response->publisher_name.data = board.publisher_name;
    //response->subscriber_name.data = board.subscriber_name;
    //response->service_name.data = board.service_name;
    //response->node_error = board.node_error;

    //response->use_wifi = board.use_wifi;
    //response->agent_ip.data = board.agent_ip;
    //response->agent_port.data = board.agent_port;
    //response->wifi_ssid.data = board.wifi_ssid;
    //response->wifi_pw.data = board.wifi_pw;
    //response->transport_error = board.transport_error;
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

    is_exec_init = false;
    is_pub_init = false;
    is_sub_init = false;
    is_service_init = false;
    is_node_init = false;
    is_supp_init = false;

    usleep(1e6); // TODO: Remove

    printf(">>> Init options\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	OK_OR_CLEANUP(rcl_init_options_init(&init_options, allocator));

    printf(">>> Init RMW\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    printf("%s:%s\n", board.agent_ip, board.agent_port);

	// Static Agent IP and port can be used instead of autodisvery.
	OK_OR_CLEANUP(rmw_uros_options_set_udp_address(board.agent_ip, board.agent_port, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
    
    printf(">>> Init support\n");
	OK_OR_CLEANUP(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    is_supp_init = true;

    // --- Create node ---

    printf(">>> Init node\n");
    OK_OR_CLEANUP(rclc_node_init_default(&node, board.node_name, "", &support));
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.publisher_name
    ));
    is_pub_init = true;

    // --- Create subscriber ---
    
    printf(">>> Init subscriber\n");
    subscriber = rcl_get_zero_initialized_subscription();
    OK_OR_CLEANUP(rclc_subscription_init_best_effort(
        &subscriber, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        board.subscriber_name
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
        ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_esp32_interfaces, srv, SetConfig), 
        board.service_name
    ));
    
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
    const unsigned int timer_timeout = board.refresh_rate_ms;
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
    //    //rcl_ret_t err = rclc_executor_spin_some(&executor, 100);
    //      rcl_ret_t rc = RCL_RET_OK;
    //      //RCL_CHECK_ARGUMENT_FOR_NULL(&executor, RCL_RET_INVALID_ARGUMENT);
    //      rc = check(&executor, RCL_RET_INVALID_ARGUMENT);

    //      printf("%d\n", rcl_context_is_valid(executor.context));

    //      //rc = rclc_executor_prepare(&executor);

    //      if (!rcl_wait_set_is_valid(&executor.wait_set)) {
    //        // calling wait_set on zero_initialized wait_set multiple times is ok.
    //        rc = rcl_wait_set_fini(&executor.wait_set);
    //        if (rc != RCL_RET_OK) {
    //            printf("error fini: %d\n", rc);
    //        }
    //        // initialize wait_set
    //        executor.wait_set = rcl_get_zero_initialized_wait_set();
    //        // create sufficient memory space for all handles in the wait_set
    //        printf("context: %d\n", rcl_context_is_valid(executor.context));
    //        rc = rcl_wait_set_init(
    //          &executor.wait_set, executor.info.number_of_subscriptions,
    //          executor.info.number_of_guard_conditions, executor.info.number_of_timers,
    //          executor.info.number_of_clients, executor.info.number_of_services,
    //          executor.info.number_of_events,
    //          executor.context,
    //          *executor.allocator);

    //        if (rc != RCL_RET_OK) {
    //          printf("error init: %d\n", rc);
    //        }
    //      }

    //    printf("error: %d\n", rc);
    //    break;

    //    usleep(500);
    //    //printf("after sleep: %d\n", rcl_context_is_valid(board.executor.context));
    //}
    rclc_executor_spin(&executor);

cleanup:
    return node_shutdown();
}



