#include "fusb302b-wrapper.h"

#include <esp_log.h>
#include <fusb302b.h>
#include <policy_engine.h>

#define TAG "fusb302b"

#define I2C_MASTER_TIMEOUT_MS 100
#define FUSB302B_ADDR 0x22

#define MIN_CURRENT_MA 100
#define MAX_CURRENT_MA 1000

#define MAX_VOLTAGE_MV 20000

#define DPM_MIN_CURRENT PD_MA2PDI(MIN_CURRENT_MA)
#define DPM_MAX_CURRENT PD_MA2PDI(MAX_CURRENT_MA)
#define MAX_WATTAGE 20

struct fusb302b_t {
    fusb302b_t(
        PolicyEngine pe)
        : pe(pe)
    {
    }

    PolicyEngine pe;
};

extern "C" {

void fusb302b_delay(uint32_t delayms)
{
  TickType_t ticks = delayms / portTICK_PERIOD_MS;

  vTaskDelay(ticks ? ticks : 1); /* Minimum delay = 1 tick */
}

uint32_t fusb302b_timestamp(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

bool fusb302b_i2c_write(const uint8_t device_addr, const uint8_t register_add,
    const uint8_t size, uint8_t* buf)
{
    uint8_t* full_buf = (uint8_t*)malloc((size_t)size + 1);
    full_buf[0] = register_add;
    memcpy(&full_buf[1], buf, size);

    bool result = i2c_master_write_to_device((i2c_port_t)device_addr, FUSB302B_ADDR, full_buf, size + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK;

    free(full_buf);

    return result;
}

bool fusb302b_i2c_read(const uint8_t device_addr, const uint8_t register_add,
    const uint8_t size, uint8_t* buf)
{
    if (i2c_master_write_to_device((i2c_port_t)device_addr, FUSB302B_ADDR, &register_add, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) {
        return false;
    }

    return i2c_master_read_from_device((i2c_port_t)device_addr, FUSB302B_ADDR, buf, size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK;
}

bool pdbs_dpm_evaluate_capability(const pd_msg* capabilities, pd_msg* request)
{
    /* Get the number of PDOs */
    uint8_t numobj = PD_NUMOBJ_GET(capabilities);

    /* Get whether or not the power supply is constrained */

    /* Make sure we have configuration */
    /* Look at the PDOs to see if one matches our desires */
    // Look against USB_PD_Desired_Levels to select in order of preference
    uint8_t bestIndex = 0xFF;
    int bestIndexVoltage = 0;
    int bestIndexCurrent = 0;
    bool bestIsPPS = false;
    for (uint8_t i = 0; i < numobj; i++) {
        /* If we have a fixed PDO, its V equals our desired V, and its I is
         * at least our desired I */
        if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
            // This is a fixed PDO entry
            // Evaluate if it can produve sufficient current based on the
            // tipResistance (ohms*10) V=I*R -> V/I => minimum resistance, if our tip
            // resistance is >= this then we can use this supply

            int voltage_mv = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(capabilities->obj[i])); // voltage in mV units
            int current_a_x100 = PD_PDO_SRC_FIXED_CURRENT_GET(capabilities->obj[i]); // current in 10mA units
            if (voltage_mv > bestIndexVoltage || bestIndex == 0xFF) {
                // Higher voltage and valid, select this instead
                bestIndex = i;
                bestIndexVoltage = voltage_mv;
                bestIndexCurrent = current_a_x100;
                bestIsPPS = false;
            }
        } else if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (capabilities->obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS) {
            // If this is a PPS slot, calculate the max voltage in the PPS range that
            // can we be used and maintain
            uint16_t max_voltage = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(capabilities->obj[i]));
            // uint16_t min_voltage =
            // PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(capabilities->obj[i]));
            uint16_t max_current = PD_PAI2CA(PD_APDO_PPS_CURRENT_GET(capabilities->obj[i])); // max current in 10mA units
            // Using the current and tip resistance, calculate the ideal max voltage
            // if this is range, then we will work with this voltage
            // if this is not in range; then max_voltage can be safely selected
            if (max_voltage > bestIndexVoltage || bestIndex == 0xFF) {
                bestIndex = i;
                bestIndexVoltage = max_voltage;
                bestIndexCurrent = max_current;
                bestIsPPS = true;
            }
        }
    }
    if (bestIndex != 0xFF) {
        /* We got what we wanted, so build a request for that */
        request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
        if (bestIsPPS) {
            request->obj[0] = PD_RDO_PROG_CURRENT_SET(PD_CA2PAI(bestIndexCurrent)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(bestIndexVoltage)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
        } else {
            request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(bestIndexCurrent) | PD_RDO_FV_CURRENT_SET(bestIndexCurrent) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
        }
        // USB Data
        request->obj[0] |= PD_RDO_USB_COMMS;
    } else {
        /* Nothing matched (or no configuration), so get 5 V at low current */
        request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
        request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_FV_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);
        /* If the output is enabled and we got here, it must be a capability
         * mismatch. */
        if (false /*TODO: Check if you have already negotiated*/) {
            request->obj[0] |= PD_RDO_CAP_MISMATCH;
        }
        // USB Data
        request->obj[0] |= PD_RDO_USB_COMMS;
    }

    ESP_LOGI(TAG, "Evaluated");
    return true;
}

bool pdbs_dpm_epr_evaluate_capability(const epr_pd_msg* capabilities, pd_msg* request)
{
    request->hdr = PD_MSGTYPE_EPR_REQUEST | PD_NUMOBJ(2);
    request->obj[1] = capabilities->obj[0]; // Copy PDO into slot 2
    request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_FV_CURRENT_SET(DPM_MIN_CURRENT) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);

    // request->obj[0] = PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(MAX_VOLTAGE_MV)) | PD_RDO_PROG_CURRENT_SET(PD_MA2PAI(DPM_MAX_CURRENT)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);

    request->obj[0] |= PD_RDO_EPR_CAPABLE;
    // USB Data
    request->obj[0] |= PD_RDO_USB_COMMS;

    ESP_LOGI(TAG, "EPR evaluated");
    return true;
}

void pdbs_dpm_get_sink_capability(pd_msg* cap, const bool isPD3)
{
    // /* Keep track of how many PDOs we've added */
    // int numobj = 0;

    // // Must always have a PDO object for vSafe5V, indicate the bare minimum power required
    // /* Minimum current, 5 V, and higher capability. */
    // cap->obj[numobj++] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(5000)) | PD_PDO_SNK_FIXED_CURRENT_SET(DPM_MIN_CURRENT);

    // if (true) { // If requesting more than 5V
    //     /* Get the current we want */
    //     uint16_t voltage = 20 * 1000; // in mv => 20V
    //     uint16_t current = 2 * 100; // In centi-amps => 2A

    //     /* Add a PDO for the desired power. */
    //     cap->obj[numobj++] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(voltage)) | PD_PDO_SNK_FIXED_CURRENT_SET(current);

    //     /* If we want more than 5 V, set the Higher Capability flag */
    //     if (PD_MV2PDV(voltage) != PD_MV2PDV(5000)) {
    //         cap->obj[0] |= PD_PDO_SNK_FIXED_HIGHER_CAP;
    //     }
    //     /* If we're using PD 3.0, add a PPS APDO for our desired voltage */
    //     if (isPD3) {
    //         cap->obj[numobj++]
    //             = PD_PDO_TYPE_AUGMENTED | PD_APDO_TYPE_PPS | PD_APDO_PPS_MAX_VOLTAGE_SET(PD_MV2PAV(voltage)) | PD_APDO_PPS_MIN_VOLTAGE_SET(PD_MV2PAV(voltage)) | PD_APDO_PPS_CURRENT_SET(PD_CA2PAI(current));
    //     }
    // }
    // /* Set the USB communications capable flag. */
    // cap->obj[0] |= PD_PDO_SNK_FIXED_USB_COMMS;
    // // if this device is unconstrained, set the flag
    // cap->obj[0] |= PD_PDO_SNK_FIXED_UNCONSTRAINED;

    // /* Set the Sink_Capabilities message header */
    // cap->hdr = PD_MSGTYPE_SINK_CAPABILITIES | PD_NUMOBJ(numobj);

    // ESP_LOGI(TAG, "Sink capabilities fetched");
}

esp_err_t fusb302b_init(fusb302b_ptr_t* state, i2c_port_t port)
{
    FUSB302 fusb(port, fusb302b_i2c_read, fusb302b_i2c_write, fusb302b_delay);

    if (!fusb.fusb_read_id() || !fusb.fusb_setup()) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *state = new fusb302b_t(
        PolicyEngine(
            fusb,
            fusb302b_timestamp,
            fusb302b_delay,
            pdbs_dpm_get_sink_capability,
            pdbs_dpm_evaluate_capability,
            pdbs_dpm_epr_evaluate_capability,
            MAX_WATTAGE));

    ESP_LOGI(TAG, "Init OK");

    return ESP_OK;
}

esp_err_t fusb302b_poll(fusb302b_ptr_t state, bool irq_occurred)
{
    // ESP_LOGI(TAG, "Poll %u %u", state->pe.pdHasNegotiated(), state->pe.currentStateCode());

    fusb302b_timer_callback(state);

    // }   // if (!state->pe.pdHasNegotiated() && state->pe.currentStateCode(true) == 0) {
    //     state->pe.renegotiate();
 

    if (irq_occurred) {
        state->pe.IRQOccured();
    }

    while (state->pe.thread()) {
        taskYIELD();
    }

    return ESP_OK;
}

esp_err_t fusb302b_timer_callback(fusb302b_ptr_t state) {
    state->pe.TimersCallback();
    
    return ESP_OK;
}


}
