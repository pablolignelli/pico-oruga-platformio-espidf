#include "infrastructure.h"
#include <cassert>
#include "driver/gpio.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "picoros.h"
#include "picorosso.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>

/** Time to let bounce settle before re-reading the level */
#define DEBOUNCE_SETTLE_MS 20

#define BATTERY_READER_PERIOD_MS 1000
/** Raw samples averaged per battery reading */
#define BATTERY_READER_SAMPLES 16

#define ADC_MAX_RAW 4095
/** Approximate full scale at 12 dB attenuation, used only if calibration is unavailable */
#define ADC_UNCALIBRATED_FULL_SCALE_MV 3100
/** Reference voltage used by line fitting when the chip has no Vref burned in eFuse */
#define ADC_DEFAULT_VREF_MV 1100

#define BATTERY_READER_TASK_NAME "battery_read_task"
#define STOP_BUTTON_TASK_NAME "estop_button_task"
#define BATTERY_READER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define STOP_BUTTON_TASK_PRIORITY 10
#define BATTERY_PUBLISHER_BUF_SIZE 256


static const char *TAG = "infrastructure";

static gpio_num_t emergency_stop_button;
static gpio_num_t adc_battery_reader;
static float battery_divider_ratio;
static TaskHandle_t emergency_stop_task_handle;
static Infrastructure::EmergencyStopCallback emergency_stop_state_cb;

static adc_channel_t channel;
static adc_oneshot_unit_handle_t handle;
static adc_cali_handle_t adc_calibration_handle;

static picoros_publisher_t publisher_emergency_stop = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Bool),
        .rihs_hash = ROSTYPE_HASH(ros_Bool)
    }
};

static uint8_t publisher_buf[BATTERY_PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

static picoros_publisher_t publisher_battery = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_BatteryState),
        .rihs_hash = ROSTYPE_HASH(ros_BatteryState)
    }
};

// Only the voltage is measured; unmeasured fields are NaN as sensor_msgs/BatteryState specifies.
static ros_BatteryState msg_battery = {
    .header = {.frame_id = (char *)"base_link"},
    .voltage = NAN,
    .temperature = NAN,
    .current = NAN,
    .charge = NAN,
    .capacity = NAN,
    .design_capacity = NAN,
    .percentage = NAN,
    .power_supply_status = 0,     // POWER_SUPPLY_STATUS_UNKNOWN
    .power_supply_health = 0,     // POWER_SUPPLY_HEALTH_UNKNOWN
    .power_supply_technology = 0, // POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    .present = true,
    .cell_voltage = {.data = NULL, .n_elements = 0},
    .cell_temperature = {.data = NULL, .n_elements = 0},
    .location = (char *)"",
    .serial_number = (char *)"",
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

/** Averages several raw samples to smooth out ADC noise. @return pin voltage in mV, or -1 on error. */
static int read_pin_mv() {
    int raw_sum = 0;
    for (int i = 0; i < BATTERY_READER_SAMPLES; i++) {
        int raw;
        esp_err_t err = adc_oneshot_read(handle, channel, &raw);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed (%s)", esp_err_to_name(err));
            return -1;
        }
        raw_sum += raw;
    }
    int raw = raw_sum / BATTERY_READER_SAMPLES;

    int mv;
    if (adc_calibration_handle == NULL || adc_cali_raw_to_voltage(adc_calibration_handle, raw, &mv) != ESP_OK) {
        mv = raw * ADC_UNCALIBRATED_FULL_SCALE_MV / ADC_MAX_RAW; // rough estimate
    }
    return mv;
}

static void battery_voltage_reader_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        int pin_mv = read_pin_mv();
        if (pin_mv >= 0) {
            z_clock_t now;
            clock_gettime(CLOCK_REALTIME, &now);
            PicoRosso::set_timestamp(msg_battery.header.stamp, now);
            msg_battery.voltage = pin_mv / 1000.0f * battery_divider_ratio;
            ESP_LOGD(TAG, "Battery: %d mV at pin, %.2f V", pin_mv, msg_battery.voltage);
            pr_publish_buf(publisher_battery, msg_battery, publisher_buf, sizeof(publisher_buf));
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BATTERY_READER_PERIOD_MS));
    }
}

static bool setup_adc() {
    adc_unit_t unit_id;
    esp_err_t err = adc_oneshot_io_to_channel(adc_battery_reader, &unit_id, &channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d is not an ADC pin", adc_battery_reader);
        return false;
    }
    if (unit_id != ADC_UNIT_1) {
        // ADC2 is shared with the WiFi driver and reads fail while it is active
        ESP_LOGW(TAG, "GPIO %d is on ADC2, reads may fail while WiFi is on", adc_battery_reader);
    }

    adc_oneshot_unit_init_cfg_t unit_config = {.unit_id = unit_id, .ulp_mode = ADC_ULP_MODE_DISABLE};
    err = adc_oneshot_new_unit(&unit_config, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not init ADC unit (%s)", esp_err_to_name(err));
        return false;
    }

    adc_oneshot_chan_cfg_t channel_config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    err = adc_oneshot_config_channel(handle, channel, &channel_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure ADC channel (%s)", esp_err_to_name(err));
        return false;
    }

    adc_cali_line_fitting_config_t calibration_config = {.unit_id = unit_id,
                                                         .atten = ADC_ATTEN_DB_12,
                                                         .bitwidth = ADC_BITWIDTH_12,
                                                         .default_vref = ADC_DEFAULT_VREF_MV};
    err = adc_cali_create_scheme_line_fitting(&calibration_config, &adc_calibration_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s), readings will be approximate", esp_err_to_name(err));
        adc_calibration_handle = NULL;
    }

    return true;
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

    // No gpio_config() for the battery pin: the ADC driver sets it up, and an
    // internal pull resistor would load the voltage divider and skew readings.
    return setup_adc();
}

bool Infrastructure::setup(gpio_num_t emergency_stop_button,
                           EmergencyStopCallback on_emergency_stop,
                           gpio_num_t adc_battery_reader,
                           float battery_divider_ratio,
                           const char *topic_emergency_stop,
                           const char *topic_battery) {

    ESP_LOGD(TAG, "Setting up...");

    assert(on_emergency_stop && "Infrastructure::setup requires an emergency stop callback");

    ::emergency_stop_button = emergency_stop_button;
    emergency_stop_state_cb = on_emergency_stop;

    ::adc_battery_reader = adc_battery_reader;
    ::battery_divider_ratio = battery_divider_ratio;

    if (!setup_gpio()) {
        ESP_LOGE(TAG, "Could not setup infrastructure module.");
        return false;
    }

    publisher_emergency_stop.topic.name = topic_emergency_stop;
    ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_emergency_stop.topic.name);
    picoros_publisher_declare(&PicoRosso::node, &publisher_emergency_stop);

    publisher_battery.topic.name = topic_battery;
    ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_battery.topic.name);
    picoros_publisher_declare(&PicoRosso::node, &publisher_battery);

    // Started only once its publisher is declared
    xTaskCreate(battery_voltage_reader_task, BATTERY_READER_TASK_NAME, 8192, NULL, BATTERY_READER_TASK_PRIORITY,
                NULL);

    ESP_LOGD(TAG, "Setting up done.");
    return true;
}

void Infrastructure::trigger_emergency_stop(bool stopped) {
    emergency_stop_state_cb(stopped);
    pr_publish(publisher_emergency_stop, stopped);
}
