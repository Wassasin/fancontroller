#pragma once

#include <driver/i2c.h>
#include <esp_err.h>

typedef struct {
    i2c_port_t port;
    uint8_t counter;
} fusb302b_state_t;

esp_err_t fusb302b_init(fusb302b_state_t* state, i2c_port_t port);
esp_err_t fusb302b_try_autoconnect(fusb302b_state_t* state);
esp_err_t fusb302b_psu_soft_reset(fusb302b_state_t* state);
esp_err_t fusb302b_poke(fusb302b_state_t* state);
esp_err_t fusb302b_poll(fusb302b_state_t* state);
