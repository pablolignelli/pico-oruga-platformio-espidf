#include "infrastructure.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "picoros.h"
#include "picorosso.h"
#include "picoserdes.h"

/** Time to let contact bounce settle before re-reading the level */
#define DEBOUNCE_SETTLE_MS 20

static const char *TAG = "infrastructure";

static gpio_num_t motor_emergency_stop;
static gpio_num_t emergency_stop_button;
static TaskHandle_t emergency_stop_task_handle;

static void emergency_stop_cb(uint8_t *rx_data, size_t data_len);

static picoros_publisher_t publisher_emergency_stop = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Bool),
        .rihs_hash = ROSTYPE_HASH(ros_Bool)
    }
};

static picoros_subscriber_t subscription_emergency_stop = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Bool),
        .rihs_hash = ROSTYPE_HASH(ros_Bool)
    },
    .user_callback = emergency_stop_cb
};

static void toggle_emergency_stop(bool toggled) {
    gpio_set_level(motor_emergency_stop, toggled ? 0 : 1);
}

static void emergency_stop_cb(uint8_t *rx_data, size_t data_len) {
    bool received_stop;

    if (!ps_deserialize(rx_data, &received_stop, data_len)) {
        ESP_LOGE(TAG, "emergency stop message deserialization error");
        return;
    }

    ESP_LOGI(TAG, "Received emergency stop message: %d", received_stop);

    toggle_emergency_stop(received_stop);
}

static void IRAM_ATTR emergency_stop_button_isr(void *arg) {
    gpio_num_t gpio_num = (gpio_num_t)(intptr_t)arg;
    if (gpio_get_level(gpio_num) != 0) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(emergency_stop_task_handle,
                           &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void emergency_stop_button_task(void *arg) {
    bool was_pressed = false;
    bool stopped = false;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // sleeps here until the ISR fires

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_SETTLE_MS)); // let contact bounce settle
        bool pressed = gpio_get_level(emergency_stop_button) == 0;

        // TODO: Currently the button works as a toggler instead of the emergency stop being on while pressed.
        // Should directly use button's pressed state when using an actual emergency stop button.
        if (pressed && !was_pressed) {
            stopped = !stopped;
            Infrastructure::trigger_emergency_stop(stopped);
        }
        was_pressed = pressed;
    }
}

static bool setup_gpio() {
    gpio_config_t motor_emergency_stop_conf = {
        .pin_bit_mask = (1ULL << motor_emergency_stop),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&motor_emergency_stop_conf);
    gpio_set_level(motor_emergency_stop, 1);

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

static void setup_topics(const char *topic_emergency_stop) {
    publisher_emergency_stop.topic.name = topic_emergency_stop;
    ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_emergency_stop.topic.name);
    picoros_publisher_declare(&PicoRosso::node, &publisher_emergency_stop);

    subscription_emergency_stop.topic.name = topic_emergency_stop;
    ESP_LOGI(TAG, "Declaring subscription on [%s]", subscription_emergency_stop.topic.name);
    picoros_subscriber_declare(&PicoRosso::node, &subscription_emergency_stop);
}

bool Infrastructure::setup(gpio_num_t motor_emergency_stop,
                           gpio_num_t emergency_stop_button,
                           const char *topic_emergency_stop) {

    ESP_LOGD(TAG, "Setting up...");

    ::motor_emergency_stop = motor_emergency_stop;
    ::emergency_stop_button = emergency_stop_button;

    if (!setup_gpio()) {
        ESP_LOGE(TAG, "Could not setup infrastructure module.");
        return false;
    }

    setup_topics(topic_emergency_stop);

    ESP_LOGD(TAG, "Setting up done.");
    return true;
}

void Infrastructure::trigger_emergency_stop(bool toggled) {
    toggle_emergency_stop(toggled);
    pr_publish(publisher_emergency_stop, toggled);
}
