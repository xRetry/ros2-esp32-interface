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
#include <unistd.h>

#include "modes.h"
#include "ros2_node.h"
#include "stdbool.h"
#include "tracetools/tracetools.h"

#include <rcl/publisher.h>

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

struct rcl_publisher_impl_s
{
  rcl_publisher_options_t options;
  rmw_qos_profile_t actual_qos;
  rcl_context_t * context;
  rmw_publisher_t * rmw_handle;
};

//rmw_publisher_t *
//rmw_create_publisher2(
//  const rmw_node_t * node,
//  const rosidl_message_type_support_t * type_support,
//  const char * topic_name,
//  const rmw_qos_profile_t * qos_policies,
//  const rmw_publisher_options_t * publisher_options)
//{
//  (void)publisher_options;
//
//  rmw_uxrce_publisher_t * custom_publisher = NULL;
//  rmw_publisher_t * rmw_publisher = NULL;
//  if (!node) {
//    RMW_UROS_TRACE_MESSAGE("node handle is null")
//  } else if (!type_support) {
//    RMW_UROS_TRACE_MESSAGE("type support is null")
//  } else if (!is_uxrce_rmw_identifier_valid(node->implementation_identifier)) {
//    RMW_UROS_TRACE_MESSAGE("node handle not from this implementation")
//  } else if (!topic_name || strlen(topic_name) == 0) {
//    RMW_UROS_TRACE_MESSAGE("publisher topic is null or empty string")
//  } else if (!qos_policies) {
//    RMW_UROS_TRACE_MESSAGE("qos_profile is null")
//  } else {
//    rmw_uxrce_node_t * custom_node = (rmw_uxrce_node_t *)node->data;
//    rmw_uxrce_mempool_item_t * memory_node = get_memory(&publisher_memory);
//    if (!memory_node) {
//      RMW_UROS_TRACE_MESSAGE("Not available memory node")
//      return NULL;
//    }
//    custom_publisher = (rmw_uxrce_publisher_t *)memory_node->data;
//
//    rmw_publisher = &custom_publisher->rmw_publisher;
//    rmw_publisher->data = custom_publisher;
//    rmw_publisher->implementation_identifier = rmw_get_implementation_identifier();
//    rmw_publisher->topic_name = custom_publisher->topic_name;
//
//    if ((strlen(topic_name) + 1 ) > sizeof(custom_publisher->topic_name)) {
//      RMW_UROS_TRACE_MESSAGE("failed to allocate string")
//      goto fail;
//    }
//    snprintf(
//      (char *)rmw_publisher->topic_name, sizeof(custom_publisher->topic_name), "%s",
//      topic_name);
//
//    custom_publisher->owner_node = custom_node;
//    custom_publisher->session_timeout = RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT;
//    custom_publisher->qos = *qos_policies;
//
//    custom_publisher->stream_id =
//      (qos_policies->reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) ?
//      custom_node->context->best_effort_output :
//      custom_node->context->reliable_output;
//
//    custom_publisher->cs_cb_size = NULL;
//    custom_publisher->cs_cb_serialization = NULL;
//
//    const rosidl_message_type_support_t * type_support_xrce = NULL;
//#ifdef ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE
//    type_support_xrce = get_message_typesupport_handle(
//      type_support, ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE);
//#endif /* ifdef ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE */
//#ifdef ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP__IDENTIFIER_VALUE
//    if (NULL == type_support_xrce) {
//      type_support_xrce = get_message_typesupport_handle(
//        type_support, ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP__IDENTIFIER_VALUE);
//    }
//#endif /* ifdef ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP__IDENTIFIER_VALUE */
//    if (NULL == type_support_xrce) {
//      RMW_UROS_TRACE_MESSAGE("Undefined type support")
//      goto fail;
//    }
//
//    custom_publisher->type_support_callbacks =
//      (const message_type_support_callbacks_t *)type_support_xrce->data;
//
//    if (custom_publisher->type_support_callbacks == NULL) {
//      RMW_UROS_TRACE_MESSAGE("type support data is NULL")
//      goto fail;
//    }
//
//    // Create topic
//    custom_publisher->topic = create_topic(
//      custom_node, topic_name,
//      custom_publisher->type_support_callbacks, qos_policies);
//
//    if (custom_publisher->topic == NULL) {
//      RMW_UROS_TRACE_MESSAGE("Error creating topic")
//      goto fail;
//    }
//
//    // Create publisher
//    custom_publisher->publisher_id = uxr_object_id(
//      custom_node->context->id_publisher++,
//      UXR_PUBLISHER_ID);
//    uint16_t publisher_req = UXR_INVALID_REQUEST_ID;
//
//  #ifdef RMW_UXRCE_USE_REFS
//    publisher_req = uxr_buffer_create_publisher_xml(
//      &custom_publisher->owner_node->context->session,
//      *custom_node->context->creation_stream,
//      custom_publisher->publisher_id,
//      custom_node->participant_id, "", UXR_REPLACE | UXR_REUSE);
//  #else
//    publisher_req = uxr_buffer_create_publisher_bin(
//      &custom_publisher->owner_node->context->session,
//      *custom_node->context->creation_stream,
//      custom_publisher->publisher_id,
//      custom_node->participant_id,
//      UXR_REPLACE | UXR_REUSE);
//  #endif /* ifdef RMW_UXRCE_USE_REFS */
//
//    if (!run_xrce_session(
//        custom_node->context, custom_node->context->creation_stream, publisher_req,
//        custom_node->context->creation_timeout))
//    {
//      goto fail;
//    }
//
//    // Create datawriter
//    custom_publisher->datawriter_id = uxr_object_id(
//      custom_node->context->id_datawriter++,
//      UXR_DATAWRITER_ID);
//    uint16_t datawriter_req = UXR_INVALID_REQUEST_ID;
//
//  #ifdef RMW_UXRCE_USE_REFS
//    if (!build_datawriter_profile(
//        topic_name, rmw_uxrce_entity_naming_buffer,
//        sizeof(rmw_uxrce_entity_naming_buffer)))
//    {
//      RMW_UROS_TRACE_MESSAGE("failed to generate xml request for node creation")
//      goto fail;
//    }
//
//    datawriter_req = uxr_buffer_create_datawriter_ref(
//      &custom_publisher->owner_node->context->session,
//      *custom_node->context->creation_stream,
//      custom_publisher->datawriter_id,
//      custom_publisher->publisher_id, rmw_uxrce_entity_naming_buffer, UXR_REPLACE | UXR_REUSE);
//  #else
//    datawriter_req = uxr_buffer_create_datawriter_bin(
//      &custom_publisher->owner_node->context->session,
//      *custom_node->context->creation_stream,
//      custom_publisher->datawriter_id,
//      custom_publisher->publisher_id,
//      custom_publisher->topic->topic_id,
//      convert_qos_profile(qos_policies),
//      UXR_REPLACE | UXR_REUSE);
//  #endif /* ifdef RMW_UXRCE_USE_REFS */
//
//    if (!run_xrce_session(
//        custom_node->context, custom_node->context->creation_stream, datawriter_req,
//        custom_node->context->creation_timeout))
//    {
//      goto fail;
//    }
//  }
//
//  return rmw_publisher;
//fail:
//  if (custom_publisher != NULL && custom_publisher->topic != NULL) {
//    rmw_uxrce_fini_topic_memory(custom_publisher->topic);
//  }
//
//  rmw_uxrce_fini_publisher_memory(rmw_publisher);
//  rmw_publisher = NULL;
//  return rmw_publisher;
//}

rcl_ret_t
rcl_publisher_init2(
  rcl_publisher_t * publisher,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rcl_publisher_options_t * options
)
{
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_INVALID_ARGUMENT);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_ALREADY_INIT);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_NODE_INVALID);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_BAD_ALLOC);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_ERROR);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_TOPIC_NAME_INVALID);

  rcl_ret_t fail_ret = RCL_RET_ERROR;

  // Check options and allocator first, so allocator can be used with errors.
  RCL_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
  rcl_allocator_t * allocator = (rcl_allocator_t *)&options->allocator;
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  RCL_CHECK_ARGUMENT_FOR_NULL(publisher, RCL_RET_INVALID_ARGUMENT);
  if (publisher->impl) {
    RCL_SET_ERROR_MSG("publisher already initialized, or memory was unintialized");
    return RCL_RET_ALREADY_INIT;
  }
  if (!rcl_node_is_valid(node)) {
    return RCL_RET_NODE_INVALID;  // error already set
  }
  RCL_CHECK_ARGUMENT_FOR_NULL(type_support, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(topic_name, RCL_RET_INVALID_ARGUMENT);
  RCUTILS_LOG_DEBUG_NAMED(
    ROS_PACKAGE_NAME, "Initializing publisher for topic name '%s'", topic_name);

  // Expand and remap the given topic name.
  char * remapped_topic_name = NULL;
  rcl_ret_t ret = rcl_node_resolve_name(
    node,
    topic_name,
    *allocator,
    false,
    false,
    &remapped_topic_name);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID || ret == RCL_RET_UNKNOWN_SUBSTITUTION) {
      ret = RCL_RET_TOPIC_NAME_INVALID;
    } else if (ret != RCL_RET_BAD_ALLOC) {
      ret = RCL_RET_ERROR;
    }
    goto cleanup;
  }
  RCUTILS_LOG_DEBUG_NAMED(
    ROS_PACKAGE_NAME, "Expanded and remapped topic name '%s'", remapped_topic_name);

  // Allocate space for the implementation struct.
  publisher->impl = (rcl_publisher_impl_t *)allocator->allocate(
    sizeof(rcl_publisher_impl_t), allocator->state);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    publisher->impl, "allocating memory failed", ret = RCL_RET_BAD_ALLOC; goto cleanup);

  printf("in: %d\n", ret);
  // Fill out implementation struct.
  // rmw handle (create rmw publisher)
  // TODO(wjwwood): pass along the allocator to rmw when it supports it
  publisher->impl->rmw_handle = rmw_create_publisher(
    rcl_node_get_rmw_handle(node),
    type_support,
    remapped_topic_name,
    &(options->qos),
    &(options->rmw_publisher_options));
  printf("cp: %d\n", ret);
  RCL_CHECK_FOR_NULL_WITH_MSG( // TODO: Error here
    publisher->impl->rmw_handle, rmw_get_error_string().str, goto fail);
  // get actual qos, and store it
  printf("cp2: %d\n", ret);
  rmw_ret_t rmw_ret = rmw_publisher_get_actual_qos(
    publisher->impl->rmw_handle,
    &publisher->impl->actual_qos);
  printf("in2: %d\n", rmw_ret);
  if (RMW_RET_OK != rmw_ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    goto fail;
  }
  publisher->impl->actual_qos.avoid_ros_namespace_conventions =
    options->qos.avoid_ros_namespace_conventions;
  // options
  publisher->impl->options = *options;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Publisher initialized");
  // context
  publisher->impl->context = node->context;
  TRACEPOINT(
    rcl_publisher_init,
    (const void *)publisher,
    (const void *)node,
    (const void *)publisher->impl->rmw_handle,
    remapped_topic_name,
    options->qos.depth);
  printf("in3: %d\n", rmw_ret);
  goto cleanup;
fail:
  if (publisher->impl) {
    if (publisher->impl->rmw_handle) {
      rmw_ret_t rmw_fail_ret = rmw_destroy_publisher(
        rcl_node_get_rmw_handle(node), publisher->impl->rmw_handle);
      if (RMW_RET_OK != rmw_fail_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(rmw_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
      }
    }

    allocator->deallocate(publisher->impl, allocator->state);
    publisher->impl = NULL;
  }

  ret = fail_ret;
  // Fall through to cleanup
cleanup:
  allocator->deallocate(remapped_topic_name, allocator->state);
  return ret;
}

rcl_ret_t
rclc_publisher_init2(
  rcl_publisher_t * publisher,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    type_support, "type_support is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    topic_name, "topic_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    qos_profile, "qos_profile is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  (*publisher) = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  pub_opt.qos = *qos_profile;
  rcl_ret_t rc = rcl_publisher_init2(
    publisher,
    node,
    type_support,
    topic_name,
    &pub_opt);
  if (rc != RCL_RET_OK) {
    printf("%d\n", rc);
    PRINT_RCLC_ERROR(rclc_publisher_init_best_effort, rcl_publisher_init);
  }
  return rc;
}

rcl_ret_t
rclc_publisher_init_best_effort2(
  rcl_publisher_t * publisher,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name)
{
  return rclc_publisher_init2(
    publisher, node, type_support, topic_name,
    &rmw_qos_profile_sensor_data);
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

    for (int i=0; i<NUM_PINS; i++) {
        printf("pin: %d, mode: %d\n", i, request->pin_modes[i]);
    }

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
        printf("pin_nr: %d\nmode: %d\nerr: %d\n", 
               i, response->pin_modes[i], response->pin_errors[i]);
    }
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

    bool is_init_options_init = false;
    bool is_exec_init = false;
    bool is_pub_init = false;
    bool is_sub_init = false;
    bool is_service_init = false;
    bool is_node_init = false;
    bool is_supp_init = false;
    bool is_timer_init = false;

    usleep(1e6); // TODO: Remove

    printf(">>> Init options\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	OK_OR_CLEANUP(rcl_init_options_init(&init_options, allocator));
    is_init_options_init = true;

    printf(">>> Init RMW\n");
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	//rmw_uros_discover_agent(rmw_options)
	// Static Agent IP and port can be used instead of autodisvery.
	OK_OR_CLEANUP(rmw_uros_options_set_udp_address(board.agent_ip, board.agent_port, rmw_options));
    
	//while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    //    printf(">>> Waiting for agent...\n");
    //    usleep(3e6);
    //}

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
    OK_OR_CLEANUP(rclc_publisher_init_best_effort2(
        &publisher,
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(ros2_esp32_interfaces, msg, PinValues),
        "esp32_read_pins"
        //board.publisher_name
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
    const unsigned int timer_timeout = board.refresh_rate_ms;
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
    //rmw_ret_t status = RMW_RET_OK;
    //while (1) {
    //    EXECUTE_EVERY_N_MS(1000, status = rmw_uros_ping_agent(100, 1););
    //    if (status != RMW_RET_OK) goto cleanup;
    //    rcl_ret_t err = rclc_executor_spin_some(&executor, 100);

    //    printf("error: %d\n", err);
    //    //break;

    //    usleep(1000);
    //    //printf("after sleep: %d\n", rcl_context_is_valid(board.executor.context));
    //}
    rclc_executor_spin(&executor);

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
    printf("%d\n", err);
    if (is_node_init) err += rcl_node_fini(&node);
    printf("%d\n", err);
    if (is_supp_init) err += rclc_support_fini(&support);
    printf("%d\n", err);
    return err == RCL_RET_OK;
}



