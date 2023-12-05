#include "fusb302b.h"

#include "fusb302b_registers.h"
#include "util.h"

#include <esp_log.h>

#define TAG "fusb302b"

#define I2C_MASTER_TIMEOUT_MS 100
#define FUSB302B_ADDR 0x22

static esp_err_t fusb302_registers_read(i2c_port_t port, uint8_t address, uint8_t* data, size_t len)
{
    esp_err_t ret;

    ERROR_CHECK_SIMPLE(i2c_master_write_to_device(port, FUSB302B_ADDR, &address, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    ERROR_CHECK_SIMPLE(i2c_master_read_from_device(port, FUSB302B_ADDR, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

    return ESP_OK;
err:
    return ret;
}

static esp_err_t fusb302_register_read(i2c_port_t port, uint8_t address, uint8_t* data)
{
    return fusb302_registers_read(port, address, data, 1);
}

static esp_err_t fusb302_register_write(i2c_port_t port, const uint8_t address, const uint8_t data)
{
    uint8_t buf[2] = { address, data };
    return i2c_master_write_to_device(port, FUSB302B_ADDR, buf, ARRAY_SIZE(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static const char* product_id_to_str(uint8_t product_id)
{
    switch (product_id) {
    case 0b00:
        return "FUSB302BMPX";
    case 0b01:
        return "FUSB302B01MPX";
    case 0b10:
        return "FUSB302B10MPX";
    case 0b11:
        return "FUSB302B11MPX";
    default:
        return "?";
    }
}

esp_err_t fusb302b_try_autoconnect(i2c_port_t port)
{
    esp_err_t ret;
    uint8_t bc_lvl1, bc_lvl2;

    // Disconnect txcc's
    reg_switches1_t switches1 = (reg_switches1_t) { .specrev = 0b01 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES1, switches1.val));

    reg_switches0_t switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 1, .meas_cc2 = 0};
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));

    reg_status0_t status0;
    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_STATUS0, &status0.val));
    bc_lvl1 = status0.bc_lvl;

    switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 0, .meas_cc2 = 1};
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));

    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_STATUS0, &status0.val));
    bc_lvl2 = status0.bc_lvl;

    switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 0, .meas_cc2 = 0};
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));

    if (bc_lvl1 == 0 && bc_lvl2 == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (bc_lvl1 > bc_lvl2) {
        switches0.meas_cc1 = 1;
        switches1.txcc1 = 1;
    } else if (bc_lvl1 < bc_lvl2) {
        switches0.meas_cc2 = 1;
        switches1.txcc2 = 1;
    } else {
        return ESP_ERR_INVALID_STATE;
    }

    switches1.auto_crc = 1;

    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES1, switches1.val));

    return ESP_OK;
err:
    return ret;
}

esp_err_t fusb302b_init(i2c_port_t port)
{
    esp_err_t ret;

    reg_reset_t reset = (reg_reset_t) { .sw_res = 1, .pd_reset = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_RESET, reset.val));

    reg_device_id_t device_id;
    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_DEVICE_ID, &device_id.val));
    ESP_LOGI(TAG, "Device ID: %s rev%u_%u", product_id_to_str(device_id.product_id), device_id.revision_id, device_id.version_id % 0b1000);

    reg_power_t power = (reg_power_t) {
        .pwr_bandgap_wake = 1,
        .pwr_receiver_ref = 1,
        .pwr_measure = 1,
        .pwr_osc = 1,
    };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_POWER, power.val));

    reg_mask_t mask = (reg_mask_t) { .val = 0xff }; // Enable all
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_MASK, mask.val));

    reg_control0_t control0 = (reg_control0_t) { .int_mask = 0, .host_cur = 0b01 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL0, control0.val));

    reg_control3_t control3 = (reg_control3_t) { .n_retries = 0b11, .auto_retry = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL3, control3.val));

    return ESP_OK;
err:
    return ret;
}
