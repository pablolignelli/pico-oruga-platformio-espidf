#include <esp_log.h>
#include <esp_err.h>

#include "picorosso.h"
#include "picorosso_bme680.h"
#include <bme680.h>

#define TSK_MINIMAL_STACK_SIZE (1024)
#define I2C0_TASK_NAME "bmp680_task"
#define I2C0_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define I2C0_TASK_PRIORITY (tskIDLE_PRIORITY + 0)
#define PUBLISHER_BUF_SIZE 1024 //TODO

// based on I2C_BME680_CONFIG_DEFAULT
#define I2C_BME680_CONFIG {                                         \
    .i2c_address = I2C_BME680_DEV_ADDR_HI,                          \
    .i2c_clock_speed = I2C_BME680_DEV_CLK_SPD,                      \
    .power_mode = BME680_POWER_MODE_FORCED,                         \
    .iir_filter = BME680_IIR_FILTER_OFF,                            \
    .pressure_oversampling = BME680_PRESSURE_OVERSAMPLING_4X,       \
    .temperature_oversampling = BME680_TEMPERATURE_OVERSAMPLING_4X, \
    .humidity_oversampling = BME680_HUMIDITY_OVERSAMPLING_4X,       \
    .gas_enabled = true,                                            \
    .heater_temperature = 300,                                      \
    .heater_duration = 300,                                         \
    .heater_profile_size = 1}

static bme680_handle_t dev_hdl;

static const char *TAG = "bme680";

static uint8_t publisher_buf[PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

static picoros_publisher_t publisher_temperature = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_Temperature),
            .rihs_hash = ROSTYPE_HASH(ros_Temperature),
        },
};
static ros_Temperature msg_temperature = {
    .header = {.frame_id = (char *)"bme680"},
    .temperature = -273.15,
    .variance = 0.0,
};

static picoros_publisher_t publisher_humidity = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_RelativeHumidity),
            .rihs_hash = ROSTYPE_HASH(ros_RelativeHumidity),
        },
};
static ros_RelativeHumidity msg_humidity = {
    .header = {.frame_id = (char *)"bme680"},
    .relative_humidity = 0.0,
    .variance = 0.0,
};

static picoros_publisher_t publisher_pressure = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_FluidPressure),
            .rihs_hash = ROSTYPE_HASH(ros_FluidPressure),
        },
};
static ros_FluidPressure msg_pressure = {
    .header = {.frame_id = (char *)"bme680"},
    .fluid_pressure = 0.0,
    .variance = 0.0,
};

static picoros_publisher_t publisher_gasr = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_Float32),
            .rihs_hash = ROSTYPE_HASH(ros_Float32),
        },
};
static ros_Float32 msg_gasr = {
    (float)0.0,
};

EnvBME680::EnvBME680() {}

static void i2c0_bme680_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    // handle sensor
    bme680_data_t data;
    z_clock_t now;
    clock_gettime(CLOCK_REALTIME, &now);
    esp_err_t result = bme680_get_data(dev_hdl, &data);
    if (result != ESP_OK)
    {
      ESP_LOGE(TAG, "device read failed (%s)", esp_err_to_name(result));
    }
    else
    {
      if (publisher_temperature.topic.name != 0 && msg_temperature.temperature != data.air_temperature)
      {
        PicoRosso::set_timestamp(msg_temperature.header.stamp, now);
        msg_temperature.temperature = data.air_temperature;
        pr_publish(publisher_temperature, msg_temperature, publisher_buf, sizeof(publisher_buf));
      }
      if (publisher_humidity.topic.name != 0 && msg_humidity.relative_humidity != data.relative_humidity)
      {
        PicoRosso::set_timestamp(msg_humidity.header.stamp, now);
        msg_humidity.relative_humidity = data.relative_humidity;
        pr_publish(publisher_humidity, msg_humidity, publisher_buf, sizeof(publisher_buf));
      }
      if (publisher_pressure.topic.name != 0 && msg_pressure.fluid_pressure != data.barometric_pressure / 100)
      {
        PicoRosso::set_timestamp(msg_pressure.header.stamp, now);
        msg_pressure.fluid_pressure = data.barometric_pressure / 100;
        pr_publish(publisher_pressure, msg_pressure, publisher_buf, sizeof(publisher_buf));
      }
      if (publisher_gasr.topic.name != 0 && msg_gasr != data.gas_resistance / 1000)
      {
        msg_gasr = data.gas_resistance / 1000;
        pr_publish(publisher_gasr, msg_gasr, publisher_buf, sizeof(publisher_buf));
      }
      /*
      ESP_LOGI(TAG, "air temperature:     %.2f °C", data.air_temperature);
      ESP_LOGI(TAG, "dewpoint temperature:%.2f °C", data.dewpoint_temperature);
      ESP_LOGI(TAG, "relative humidity:   %.2f %%", data.relative_humidity);
      ESP_LOGI(TAG, "barometric pressure: %.2f hPa", data.barometric_pressure / 100);
      ESP_LOGI(TAG, "gas resistance:      %.2f kOhms", data.gas_resistance / 1000);
      ESP_LOGI(TAG, "iaq score:           %u (%s)", data.iaq_score, "-");
      */
    }

    // pause the task per defined wait period
    vTaskDelayUntil(&last_wake_time, BME680_SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
  }

  // free resources
  bme680_delete(dev_hdl);
  vTaskDelete(NULL);
}

bool EnvBME680::setup(i2c_master_bus_handle_t &i2c0_bus_hdl,
                      const char *topic_temperature,
                      const char *topic_humidity,
                      const char *topic_pressure,
                      const char *topic_gas_resistance)
{
  ESP_LOGD(TAG, "Setting up...");

  // initialize i2c device configuration
  bme680_config_t dev_cfg = I2C_BME680_CONFIG;
  //
  // init device
  bme680_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
  if (dev_hdl == NULL)
  {
    ESP_LOGE(TAG, "handle init failed");
    return false;
  }

  if (topic_temperature != NULL)
  {
    ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_temperature);
    publisher_temperature.topic.name = topic_temperature;
    picoros_publisher_declare(&PicoRosso::node, &publisher_temperature);
  }

  if (topic_humidity != NULL)
  {
    ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_humidity);
    publisher_humidity.topic.name = topic_humidity;
    picoros_publisher_declare(&PicoRosso::node, &publisher_humidity);
  }

  if (topic_pressure != NULL)
  {
    ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_pressure);
    publisher_pressure.topic.name = topic_pressure;
    picoros_publisher_declare(&PicoRosso::node, &publisher_pressure);
  }

  if (topic_gas_resistance != NULL)
  {
    ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_gas_resistance);
    publisher_gasr.topic.name = topic_gas_resistance;
    picoros_publisher_declare(&PicoRosso::node, &publisher_gasr);
  }

  /* create task pinned to the app core */
  xTaskCreatePinnedToCore(
      i2c0_bme680_task,
      I2C0_TASK_NAME,
      I2C0_TASK_STACK_SIZE,
      NULL,
      I2C0_TASK_PRIORITY,
      NULL,
      APP_CPU_NUM);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
