#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c.h>
#include <esp_err.h>

struct fusb302b_t;
typedef struct fusb302b_t *fusb302b_ptr_t;

esp_err_t fusb302b_init(fusb302b_ptr_t *state, i2c_port_t port);
esp_err_t fusb302b_poll(fusb302b_ptr_t state, bool irq_occurred);
esp_err_t fusb302b_timer_callback(fusb302b_ptr_t state);

// esp_err_t fusb302b_try_autoconnect(fusb302b_state_t* state);
// esp_err_t fusb302b_psu_soft_reset(fusb302b_state_t* state);
// esp_err_t fusb302b_poke(fusb302b_state_t* state);

#ifdef __cplusplus
}
#endif