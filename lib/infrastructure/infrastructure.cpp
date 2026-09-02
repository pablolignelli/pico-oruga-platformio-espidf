#include "infrastructure.h"
#include <cassert>
#include "driver/gpio.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "picoros.h"
#include "picorosso.h"

/** Time to let bounce settle before re-reading the level */
#define DEBOUNCE_SETTLE_MS 20

static const char *TAG = "infrastructure";

static gpio_num_t emergency_stop_button;
static TaskHandle_t emergency_stop_task_handle;
static Infrastructure::EmergencyStopCallback emergency_stop_state_cb;

static picoros_publisher_t publisher_emergency_stop = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Bool),
        .rihs_hash = ROSTYPE_HASH(ros_Bool)
    }
};

static void IRAM_ATTR emergency_stop_button_isr(void *arg) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(emergency_stop_task_handle,
                           &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void emergency_stop_button_task(void *arg) {
    bool was_pressed = false;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Sleep until the ISR fires

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_SETTLE_MS)); // Let contact bounce settle
        bool pressed = gpio_get_level(emergency_stop_button) == 0;

        if (pressed != was_pressed) {
            Infrastructure::trigger_emergency_stop(pressed);
        }
        was_pressed = pressed;
    }
}

static bool setup_gpio() {
    gpio_config_t button_conf = {.pin_bit_mask = (1ULL << emergency_stop_button),
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&button_conf);

    xTaskCreate(emergency_stop_button_task, "estop_button_task", 8192, NULL, 10,
                &emergency_stop_task_handle);

    esp_err_t isr_service_err = gpio_install_isr_service(0);
    if (isr_service_err != ESP_OK && isr_service_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Could not register ISR service. Got unexpected error %d", isr_service_err);
        return false;
    }
    gpio_isr_handler_add(emergency_stop_button, emergency_stop_button_isr,
                         (void *)(intptr_t)emergency_stop_button);

    return true;
}

bool Infrastructure::setup(gpio_num_t emergency_stop_button,
                           EmergencyStopCallback on_emergency_stop,
                           const char *topic_emergency_stop) {

    ESP_LOGD(TAG, "Setting up...");

    assert(on_emergency_stop && "Infrastructure::setup requires an emergency stop callback");

    ::emergency_stop_button = emergency_stop_button;
    emergency_stop_state_cb = on_emergency_stop;

    if (!setup_gpio()) {
        ESP_LOGE(TAG, "Could not setup infrastructure module.");
        return false;
    }

    publisher_emergency_stop.topic.name = topic_emergency_stop;
    ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_emergency_stop.topic.name);
    picoros_publisher_declare(&PicoRosso::node, &publisher_emergency_stop);

    ESP_LOGD(TAG, "Setting up done.");
    return true;
}

void Infrastructure::trigger_emergency_stop(bool stopped) {
    emergency_stop_state_cb(stopped);
    pr_publish(publisher_emergency_stop, stopped);
}
