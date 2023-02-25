#include "callback_handlers.h"

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
