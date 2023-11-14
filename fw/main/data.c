#include "data.h"

#include <esp_app_desc.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <sdkconfig.h>

#include "adc.h"
#include "fans.h"
#include "performance.h"
#include "tacho.h"
#include "temperature.h"

#define TAG "data"

static void mac_to_str(uint8_t mac_addr[8], char (*mac_addr_str)[18])
{
    sprintf(*mac_addr_str, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0],
        mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

static char device_id[18] = { 0 };
const char* data_get_id(void)
{
    if (device_id[16] == 0) {
        uint8_t mac_addr[8];
        esp_read_mac(mac_addr, ESP_MAC_ETH);
        mac_to_str(mac_addr, &device_id);
    }
    return device_id;
}

static const char* data_reset_reason_to_str(esp_reset_reason_t reset_reason)
{
    switch (reset_reason) {
    case ESP_RST_UNKNOWN:
        return "unknown"; // Reset reason can not be determined
    case ESP_RST_POWERON:
        return "power on"; // Reset due to power-on event
    case ESP_RST_EXT:
        return "ext"; // Reset by external pin
    case ESP_RST_SW:
        return "software reset"; // Software reset via esp_restart
    case ESP_RST_PANIC:
        return "panic"; // Software reset due to exception/panic
    case ESP_RST_INT_WDT:
        return "interrupt watchdog"; // Reset (software or hardware) due to interrupt watchdog
    case ESP_RST_TASK_WDT:
        return "task watchdog"; // Reset due to task watchdog
    case ESP_RST_WDT:
        return "other watchdog"; // Reset due to other watchdogs
    case ESP_RST_DEEPSLEEP:
        return "deep sleep"; // Reset after exiting deep sleep mode
    case ESP_RST_BROWNOUT:
        return "brownout"; // Brownout reset (software or hardware)
    case ESP_RST_SDIO:
        return "sdio"; // Reset over SDIO
    default:
        ESP_LOGW("data_reset_reason_to_str", "Unexpected value for esp_reset_reason_t: %d", reset_reason);
        return "unknown";
    }
}

static void data_status_misc_to_json(cJSON* root)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    const esp_app_desc_t* app_desc = esp_app_get_description();

    char hash_buf[17];
    esp_app_get_elf_sha256(hash_buf, sizeof(hash_buf));

    // cJSON_AddNumberToObject(root, "flash_size", spi_flash_get_chip_size());
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "minimum_heap",
        esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "free_heap_internal", esp_get_free_internal_heap_size());
    cJSON_AddNumberToObject(root, "minimum_heap_internal",
        heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(root, "chip_revision", chip_info.revision);
    cJSON_AddStringToObject(root, "esp_idf_version", esp_get_idf_version());
    cJSON_AddStringToObject(root, "app_version", app_desc->version);
    cJSON_AddStringToObject(root, "app_hash", hash_buf);
    cJSON_AddStringToObject(root, "compile_date", app_desc->date);
    cJSON_AddStringToObject(root, "compile_time", app_desc->time);
    cJSON_AddNumberToObject(root, "runtime_us", esp_timer_get_time());
    cJSON_AddStringToObject(root, "reset_reason", data_reset_reason_to_str(esp_reset_reason()));
}

esp_err_t data_status_to_json(cJSON* root)
{
    cJSON_AddStringToObject(root, "id", data_get_id());

    data_status_misc_to_json(root);

    cJSON* duty_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "duty", duty_obj);
    ESP_ERROR_CHECK(data_duty_to_json(duty_obj));

    cJSON* power_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "power", power_obj);
    ESP_ERROR_CHECK(data_power_to_json(power_obj));

    cJSON* tacho_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "tacho", tacho_obj);
    ESP_ERROR_CHECK(data_tacho_to_json(tacho_obj));

    cJSON* sensors_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "sensors", sensors_obj);
    ESP_ERROR_CHECK(data_sensors_to_json(sensors_obj));

    cJSON* performance_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "performance", performance_obj);
    ESP_ERROR_CHECK(data_performance_to_json(performance_obj));

    return ESP_OK;
}

static void sample_to_json(cJSON* root, const char* name, const adc_sample_t* sample)
{
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, name, obj);

    cJSON_AddNumberToObject(obj, "rms", sample->rms);
    cJSON_AddNumberToObject(obj, "max", sample->max);
}

esp_err_t data_power_to_json(cJSON* root)
{
    adc_samples_t samples;
    adc_fetch(&samples);

    sample_to_json(root, "vbus_mv", &samples.vbus_mv);
    sample_to_json(root, "vbus_ma", &samples.vbus_ma);
    sample_to_json(root, "vfan_mv", &samples.vfan_mv);

    sample_to_json(root, "vfan1_ma", &samples.vfan_ma[0]);
    sample_to_json(root, "vfan2_ma", &samples.vfan_ma[1]);
    sample_to_json(root, "vfan3_ma", &samples.vfan_ma[2]);
    sample_to_json(root, "vfan4_ma", &samples.vfan_ma[3]);
    sample_to_json(root, "vfan5_ma", &samples.vfan_ma[4]);

    return ESP_OK;
}

esp_err_t data_duty_to_json(cJSON* root)
{
    fans_pwm8_t fans;
    fans_fetch(fans);

    cJSON_AddNumberToObject(root, "fan1_pwm8", fans[0]);
    cJSON_AddNumberToObject(root, "fan2_pwm8", fans[1]);
    cJSON_AddNumberToObject(root, "fan3_pwm8", fans[2]);
    cJSON_AddNumberToObject(root, "fan4_pwm8", fans[3]);
    cJSON_AddNumberToObject(root, "fan5_pwm8", fans[4]);

    return ESP_OK;
}

esp_err_t data_tacho_to_json(cJSON* root)
{
    tacho_fans_rpm_t rpm;
    tacho_fetch(rpm);

    cJSON_AddNumberToObject(root, "fan1_rpm", rpm[0]);
    cJSON_AddNumberToObject(root, "fan2_rpm", rpm[1]);
    cJSON_AddNumberToObject(root, "fan3_rpm", rpm[2]);
    cJSON_AddNumberToObject(root, "fan4_rpm", rpm[3]);
    cJSON_AddNumberToObject(root, "fan5_rpm", rpm[4]);

    return ESP_OK;
}

esp_err_t data_sensors_to_json(cJSON* root)
{
    temperature_sample_t sample;

    if (temperature_fetch(TEMPERATURE_CHANNEL_ON_BOARD, &sample)) {
        cJSON* on_board_obj = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "temphum_on_board", on_board_obj);

        cJSON_AddNumberToObject(on_board_obj, "temperature_mc", sample.temperature_mc);
        cJSON_AddNumberToObject(on_board_obj, "rel_hum_mperct", sample.rel_hum_mperct);
    }

    if (temperature_fetch(TEMPERATURE_CHANNEL_EXTERNAL, &sample)) {
        cJSON* external_obj = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "temphum_external", external_obj);

        cJSON_AddNumberToObject(external_obj, "temperature_mc", sample.temperature_mc);
        cJSON_AddNumberToObject(external_obj, "rel_hum_mperct", sample.rel_hum_mperct);
    }

    return ESP_OK;
}

static void data_performance_to_json_emit(performance_entry_t entry, void* ctx)
{
    cJSON* root = (cJSON*)ctx;
    cJSON_AddNumberToObject(root, entry.task_name, entry.percentage);
}

esp_err_t data_performance_to_json(cJSON* root)
{
    performance_fetch(data_performance_to_json_emit, root);
    return ESP_OK;
}

static void data_try_process_duty_json(cJSON* root, const char* key, uint8_t fan_i)
{
    cJSON* obj = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(obj)) {
        fans_command(fan_i, cJSON_GetNumberValue(obj));
    }
}

esp_err_t data_process_duty_json_str(const char* str, size_t str_len)
{
    cJSON* root = cJSON_ParseWithLength(str, str_len);

    if (root == NULL) {
        const char* error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGW(TAG, "Error before: %s", error_ptr);
        }
        return ESP_ERR_INVALID_ARG;
    }

    cJSON* obj = cJSON_GetObjectItemCaseSensitive(root, "fans_pwm8");
    if (cJSON_IsNumber(obj)) {
        for (size_t i = 0; i < FANS_COUNT; ++i) {
            fans_command(i, cJSON_GetNumberValue(obj));
        }
    }

    data_try_process_duty_json(root, "fan1_pwm8", 0);
    data_try_process_duty_json(root, "fan2_pwm8", 1);
    data_try_process_duty_json(root, "fan3_pwm8", 2);
    data_try_process_duty_json(root, "fan4_pwm8", 3);
    data_try_process_duty_json(root, "fan5_pwm8", 4);

    cJSON_Delete(root);

    return ESP_OK;
}