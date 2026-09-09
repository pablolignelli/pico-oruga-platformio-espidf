#include "infrastructure.h"
#include <cassert>
#include "driver/gpio.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "picoros.h"
#include "picorosso.h"
#include "esp_adc/adc_oneshot.h"

/** Time to let bounce settle before re-reading the level */
#define DEBOUNCE_SETTLE_MS 20

#define BATTERY_READER_PERIOD 1000

#define BATTERY_READER_TASK_NAME "battery_read_task"
#define STOP_BUTTON_TASK_NAME "stop_button_task"
#define BATTERY_READER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define STOP_BUTTON_TASK_PRIORITY 10


static const char *TAG = "infrastructure";

static gpio_num_t emergency_stop_button;
static gpio_num_t adc_battery_reader;
static TaskHandle_t emergency_stop_task_handle;
static Infrastructure::EmergencyStopCallback emergency_stop_state_cb;

static adc_channel_t channel;
static adc_oneshot_unit_handle_t handle;

static picoros_publisher_t publisher_emergency_stop = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Bool),
        .rihs_hash = ROSTYPE_HASH(ros_Bool)
    }
};

static picoros_publisher_t publisher_battery_voltage = {
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

static void battery_voltage_reader_task(void *arg) {
    while(true){
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(handle, channel, &raw));

        ESP_LOGI("adc", "Raw ADC: %d", raw);

        vTaskDelay(pdMS_TO_TICKS(BATTERY_READER_PERIOD));
    }
}

static void setup_adc() {
    adc_unit_t unit_id;
    adc_oneshot_io_to_channel(adc_battery_reader, &unit_id, &channel);

    adc_oneshot_unit_init_cfg_t config = {.unit_id=unit_id, .clk_src=ADC_RTC_CLK_SRC_DEFAULT, .ulp_mode=ADC_ULP_MODE_RISCV};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&config, &handle));
}

static bool setup_gpio() {
    gpio_config_t button_conf = {.pin_bit_mask = (1ULL << emergency_stop_button),
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&button_conf);

    xTaskCreate(emergency_stop_button_task, STOP_BUTTON_TASK_NAME, 8192, NULL, STOP_BUTTON_TASK_PRIORITY,
                &emergency_stop_task_handle);

    esp_err_t isr_service_err = gpio_install_isr_service(0);
    if (isr_service_err != ESP_OK && isr_service_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Could not register ISR service. Got unexpected error %d", isr_service_err);
        return false;
    }
    gpio_isr_handler_add(emergency_stop_button, emergency_stop_button_isr,
                         (void *)(intptr_t)emergency_stop_button);

    gpio_config_t battery_voltage_reader_conf = {   .pin_bit_mask = (1ULL << adc_battery_reader),
                                                    .mode = GPIO_MODE_INPUT,
                                                    .pull_up_en = GPIO_PULLUP_ENABLE,
                                                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                                    .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&battery_voltage_reader_conf);
    setup_adc();

    xTaskCreate(battery_voltage_reader_task, BATTERY_READER_TASK_NAME, 8192, NULL, BATTERY_READER_TASK_PRIORITY,
                &emergency_stop_task_handle);

    return true;
}

bool Infrastructure::setup(gpio_num_t emergency_stop_button,
                           EmergencyStopCallback on_emergency_stop,
                           gpio_num_t adc_battery_reader,
                           const char *topic_emergency_stop,
                           const char *topic_battery_voltage) {

    ESP_LOGD(TAG, "Setting up...");

    assert(on_emergency_stop && "Infrastructure::setup requires an emergency stop callback");

    ::emergency_stop_button = emergency_stop_button;
    emergency_stop_state_cb = on_emergency_stop;

    ::adc_battery_reader = adc_battery_reader;

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
