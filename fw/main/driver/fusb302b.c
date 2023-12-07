#include "fusb302b.h"

#include "fusb302b_pd.h"
#include "fusb302b_registers.h"

#include "led.h"
#include "util.h"

#include <esp_log.h>

#define TAG "fusb302b"

#define I2C_MASTER_TIMEOUT_MS 100
#define FUSB302B_ADDR 0x22

typedef struct {
    uint8_t index;
    bool is_pps;
    uint32_t voltage_mv;
    uint32_t current_ma;
} fusb302_pdo_t;

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

static esp_err_t fusb302_registers_write(i2c_port_t port, const uint8_t address, const uint8_t* data, size_t len)
{
    esp_err_t ret;

    // TODO bulk transfer
    for (size_t i = 0; i < len; ++i) {
        ERROR_CHECK_SIMPLE(fusb302_register_write(port, address, data[i]));
    }

    return ESP_OK;
err:
    return ret;
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

    reg_switches0_t switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 1, .meas_cc2 = 0 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));

    reg_status0_t status0;
    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_STATUS0, &status0.val));
    bc_lvl1 = status0.bc_lvl;

    switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 0, .meas_cc2 = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_SWITCHES0, switches0.val));

    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_STATUS0, &status0.val));
    bc_lvl2 = status0.bc_lvl;

    switches0 = (reg_switches0_t) { .pdwn1 = 1, .pdwn2 = 1, .meas_cc1 = 0, .meas_cc2 = 0 };
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

    reg_control0_t control0 = (reg_control0_t) { .tx_flush = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL0, control0.val));

    reg_control1_t control1 = (reg_control1_t) { .rx_flush = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL1, control1.val));

    reg_reset_t reset = (reg_reset_t) { .pd_reset = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_RESET, reset.val));

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

    reg_control0_t control0 = (reg_control0_t) {};
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL0, control0.val));

    reg_control3_t control3 = (reg_control3_t) { .n_retries = 0b11, .auto_retry = 1 };
    ERROR_CHECK_SIMPLE(fusb302_register_write(port, REG_CONTROL3, control3.val));

    return ESP_OK;
err:
    return ret;
}

static esp_err_t fusb302b_read_message(i2c_port_t port, bool* msg_filled, pd_msg* msg)
{
    esp_err_t ret;
    *msg_filled = false;

    uint8_t byte;
    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_FIFOS, &byte));

    // Only interested in SOP-messages
    if ((byte & FIFO_RX_TOKEN_BITS) != FIFO_RX_SOP) {
        return ESP_OK;
    }

    ERROR_CHECK_SIMPLE(fusb302_registers_read(port, REG_FIFOS, msg->bytes, 2));
    uint8_t numobj = PD_NUMOBJ_GET(msg);

    if (numobj > 0) {
        ERROR_CHECK_SIMPLE(fusb302_registers_read(port, REG_FIFOS, msg->bytes + 2, numobj * 4));
    }

    // Throw away CRC as the chip has already checked it.
    uint8_t garbage[4];
    ERROR_CHECK_SIMPLE(fusb302_registers_read(port, REG_FIFOS, garbage, ARRAY_SIZE(garbage)));

    *msg_filled = true;

    return ESP_OK;
err:
    return ret;
}

static esp_err_t fusb302b_send_message(i2c_port_t port, const pd_msg* msg)
{
    esp_err_t ret;

    static uint8_t sop_seq[5] = { FIFO_TX_SOP1, FIFO_TX_SOP1, FIFO_TX_SOP1, FIFO_TX_SOP2, FIFO_TX_PACKSYM };
    static const uint8_t eop_seq[4] = { FIFO_TX_JAM_CRC, FIFO_TX_EOP, FIFO_TX_TXOFF, FIFO_TX_TXON };

    uint8_t msg_len = 2 + 4 * PD_NUMOBJ_GET(msg);

    // Set number of bytes in header
    sop_seq[4] = FIFO_TX_PACKSYM | msg_len;

    ERROR_CHECK_SIMPLE(fusb302_registers_write(port, REG_FIFOS, sop_seq, ARRAY_SIZE(sop_seq)));
    ERROR_CHECK_SIMPLE(fusb302_registers_write(port, REG_FIFOS, msg->bytes, msg_len));
    ERROR_CHECK_SIMPLE(fusb302_registers_write(port, REG_FIFOS, eop_seq, ARRAY_SIZE(eop_seq)));

    return ESP_OK;
err:
    return ret;
}

static esp_err_t fusb302b_find_best_pdo(i2c_port_t port, const pd_msg* msg, fusb302_pdo_t* pdo)
{
    uint32_t best_power_mw = 0;

    uint8_t numobj = PD_NUMOBJ_GET(msg);
    for (size_t i = 0; i < numobj; ++i) {
        if ((msg->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
            uint32_t voltage_mv = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(msg->obj[i]));

            if (voltage_mv > 24000) {
                continue;
            }

            uint32_t current_ma = PD_PDI2MA(PD_PDO_SRC_FIXED_CURRENT_GET(msg->obj[i]));
            uint32_t power_mw = voltage_mv * current_ma;
            if (power_mw > best_power_mw) {
                pdo->index = i;
                pdo->voltage_mv = voltage_mv;
                pdo->current_ma = current_ma;
                pdo->is_pps = false;
            }
        } else if ((msg->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (msg->obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS) {
            uint32_t voltage_mv = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(msg->obj[i]));

            if (voltage_mv > 24000) {
                continue;
            }

            uint32_t current_ma = PD_PAI2MA(PD_APDO_PPS_CURRENT_GET(msg->obj[i]));
            uint32_t power_mw = voltage_mv * current_ma;

            if (power_mw > best_power_mw) {
                pdo->index = i;
                pdo->voltage_mv = voltage_mv;
                pdo->current_ma = current_ma;
                pdo->is_pps = true;
            }
        }
    }

    return ESP_OK;
}

static esp_err_t fusb302_select_pdo(i2c_port_t port, const fusb302_pdo_t* pdo)
{
    pd_msg msg;
    msg.hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);

    if (pdo->is_pps) {
        msg.obj[0] = PD_RDO_PROG_CURRENT_SET(PD_MA2PAI(pdo->current_ma)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(pdo->voltage_mv)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(pdo->index + 1);
    } else {
        msg.obj[0] = PD_RDO_FV_MAX_CURRENT_SET(PD_MA2CA(pdo->current_ma)) | PD_RDO_FV_CURRENT_SET(PD_MA2CA(pdo->current_ma)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(pdo->index + 1);
    }

    msg.obj[0] |= PD_RDO_USB_COMMS;

    return fusb302b_send_message(port, &msg);
}

esp_err_t fusb302b_poll(i2c_port_t port)
{
    esp_err_t ret;

    reg_status1_t status1;
    ERROR_CHECK_SIMPLE(fusb302_register_read(port, REG_STATUS1, &status1.val));

    while (status1.rx_empty == 0) {
        // Message waiting
        ESP_LOGI(TAG, "Message waiting!");

        bool msg_filled = false;
        pd_msg msg;
        ERROR_CHECK_SIMPLE(fusb302b_read_message(port, &msg_filled, &msg));

        if (!msg_filled) {
            continue;
        }

        fusb302_pdo_t pdo = (fusb302_pdo_t) { .index = 0xff };
        ERROR_CHECK_SIMPLE(fusb302b_find_best_pdo(port, &msg, &pdo));

        if (pdo.index != 0xff) {
            ERROR_CHECK_SIMPLE(fusb302_select_pdo(port, &pdo));
            led_set_color((rgb_t) {
                .r = 0x00,
                .g = 0x01,
                .b = 0x00,
            });
        }
    }

    return ESP_OK;
err:
    return ret;
}