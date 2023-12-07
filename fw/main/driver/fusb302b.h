#pragma once

#include <driver/i2c.h>
#include <esp_err.h>

esp_err_t fusb302b_init(i2c_port_t port);
esp_err_t fusb302b_try_autoconnect(i2c_port_t port);
esp_err_t fusb302b_poll(i2c_port_t port);
