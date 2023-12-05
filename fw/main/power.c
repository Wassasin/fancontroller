#include "power.h"

#include <esp_log.h>

#include "driver/fusb302b.h"
#include "i2c_bus.h"

#define TAG "power"

#define GPIO_INT_N (18)
#define I2C_BUS (I2C_BUS_PRIMARY_NUM)

static SemaphoreHandle_t s_sem;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem, &xHigherPriorityTaskWoken); // Ignore result
    if( xHigherPriorityTaskWoken == pdTRUE ) {
        portYIELD_FROM_ISR();
    }
}

static void power_task(void* arg)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_INT_N),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INT_N, gpio_isr_handler, NULL);

    // Temporary test
    ESP_ERROR_CHECK(i2c_bus_take(I2C_BUS));
    ESP_ERROR_CHECK(fusb302b_init(I2C_BUS));
    i2c_bus_give(I2C_BUS);

    bool pd = false;
    while (1) {
        if (!pd) {
            ESP_ERROR_CHECK(i2c_bus_take(I2C_BUS));
            esp_err_t res = fusb302b_try_autoconnect(I2C_BUS);
            i2c_bus_give(I2C_BUS);
            if (res == ESP_OK) {
                pd = true;
                ESP_LOGI(TAG, "USB PD CC pin found!");
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

esp_err_t power_init(void)
{
    s_sem = xSemaphoreCreateBinary();
    xTaskCreate(power_task, "power", 1024 * 4, NULL, 10, NULL);

    return ESP_OK;
}