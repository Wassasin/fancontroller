#include "tacho.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "util.h"

#define TAG "tacho"

#define GPIO_FAN1_TACHO (11)
#define GPIO_FAN2_TACHO (13)
#define GPIO_FAN3_TACHO (15)
#define GPIO_FAN4_TACHO (17)
#define GPIO_FAN5_TACHO (39)

#define TACHO_EVENTS_PER_ROTATION 4

#define TACHO_DELTA_GLITCH_FILTER_US (5000) // 5 millisecond or 12500RPM
#define TACHO_READING_MAX_AGE_US (1000000) // 1 seconds or 60RPM

typedef struct
{
    uint8_t gpio_i;
    uint64_t time_us;
} tacho_event_t;

typedef struct
{
    uint64_t delta_us;
    uint64_t last_time_us;
} tacho_fan_state_t;

static tacho_fan_state_t s_tacho_fan_state[5];
static QueueHandle_t s_event_queue = NULL;
static SemaphoreHandle_t s_mutex;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    tacho_event_t event = {
        .gpio_i = (uint32_t)arg,
        .time_us = esp_timer_get_time(),
    };

    xQueueSendFromISR(s_event_queue, &event, NULL);
}

static void tacho_task(void* arg)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_FAN1_TACHO) | (1ULL << GPIO_FAN2_TACHO) | (1ULL << GPIO_FAN3_TACHO) | (1ULL << GPIO_FAN4_TACHO) | (1ULL << GPIO_FAN5_TACHO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_FAN1_TACHO, gpio_isr_handler, (void*)0);
    gpio_isr_handler_add(GPIO_FAN2_TACHO, gpio_isr_handler, (void*)1);
    gpio_isr_handler_add(GPIO_FAN3_TACHO, gpio_isr_handler, (void*)2);
    gpio_isr_handler_add(GPIO_FAN4_TACHO, gpio_isr_handler, (void*)3);
    gpio_isr_handler_add(GPIO_FAN5_TACHO, gpio_isr_handler, (void*)4);

    tacho_event_t event;
    while (1) {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY)) {
            tacho_fan_state_t* state = &s_tacho_fan_state[event.gpio_i];
            uint64_t delta_us = event.time_us - state->last_time_us;

            if (delta_us < TACHO_DELTA_GLITCH_FILTER_US / TACHO_EVENTS_PER_ROTATION) // Glitch filter
            {
                continue; // Ignore reading
            }

            // Sole writer of state, lock others from reading from it.
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            state->delta_us = delta_us;
            state->last_time_us = event.time_us;
            xSemaphoreGive(s_mutex);
        }
    }
}

esp_err_t tacho_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_event_queue = xQueueCreate(100, sizeof(tacho_event_t));
    xTaskCreate(tacho_task, "tacho", 1024 * 4, NULL, 10, NULL);

    return ESP_OK;
}

static uint32_t us_to_rpm(uint64_t us)
{
    return (60 * 1000 * 1000 / TACHO_EVENTS_PER_ROTATION) / us;
}

void tacho_fetch(tacho_fans_rpm_t fans)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint64_t now_us = esp_timer_get_time();

    for (size_t i = 0; i < ARRAY_SIZE(s_tacho_fan_state); ++i) {
        if (
            s_tacho_fan_state[i].delta_us > 0 && s_tacho_fan_state[i].delta_us < TACHO_READING_MAX_AGE_US && s_tacho_fan_state[i].last_time_us + TACHO_READING_MAX_AGE_US > now_us) {
            fans[i] = us_to_rpm(s_tacho_fan_state[i].delta_us);
        } else {
            fans[i] = 0;
        }
    }
    xSemaphoreGive(s_mutex);
}