#ifndef CALLBACK_HANDLERS_H_   /* Include guard */
#define CALLBACK_HANDLERS_H_   /* Include guard */

#include "hardware_control.h"
#include <rcl/rcl.h>

typedef struct pin_values_t {
    double vals[NUM_PINS];
} pin_values_t;

typedef struct set_pin_req_t {
    int pin_nr;
    pin_mode_t mode;    
    pin_type_t type;
} set_pin_req_t;

typedef struct  set_pin_rsp_t {
    bool is_ok;
} set_pin_rsp_t;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
pin_values_t recv_msg;

void handle_write_pins(const void *msgin);

void handle_read_pins(rcl_timer_t *timer, int64_t last_call_time);

void handle_set_pin(const void *msg_req, void *msg_rsp);

#endif // CALLBACK_HANDLERS_H_
