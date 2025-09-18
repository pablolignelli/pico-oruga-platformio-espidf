#include <esp_log.h>
#include <esp_err.h>

#include "picorosso.h"
#include "picorosso_mpu6050.h"
#include <mpu6050.h>

#define TSK_MINIMAL_STACK_SIZE (1024)
#define I2C0_TASK_NAME "mpu6050_task"
#define I2C0_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define I2C0_TASK_PRIORITY (tskIDLE_PRIORITY + 0)

// based on I2C_MPU6050_CONFIG_DEFAULT
#define I2C_MPU6050_CONFIG {                                               \
    .i2c_address = I2C_MPU6050_DEV_ADDR_L,                                 \
    .i2c_clock_speed = I2C_MPU6050_DEV_CLK_SPD,                            \
    .low_pass_filter = MPU6050_DIGITAL_LP_FILTER_ACCEL_260KHZ_GYRO_256KHZ, \
    .gyro_clock_source = MPU6050_GYRO_CS_PLL_X_AXIS_REF,                   \
    .gyro_full_scale_range = MPU6050_GYRO_FS_RANGE_500DPS,                 \
    .accel_full_scale_range = MPU6050_ACCEL_FS_RANGE_4G}

static mpu6050_handle_t dev_hdl;

static const char *TAG = "imu";

static picoros_publisher_t publisher_raw = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_Imu),
            .rihs_hash = ROSTYPE_HASH(ros_Imu),
        },
};
static ros_Imu msg_raw = {
    .header = {.frame_id = (char *)"imu_link"},
    .orientation_covariance = {-1, -1, -1, -1, -1, -1, -1, -1, -1},
    .angular_velocity_covariance = {0.05, 0, 0, 0, 0.5, 0, 0, 0, 0.5},
    .linear_acceleration_covariance = {0.05, 0, 0, 0, 0.5, 0, 0, 0, 0.5},
};

static picoros_publisher_t publisher_temperature = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Temperature),
        .rihs_hash = ROSTYPE_HASH(ros_Temperature),
    },
};
static ros_Temperature msg_temperature = {
    .header = {.frame_id = (char *)"imu_link"},
    .temperature = -273.15,
    .variance = 0.0,
};

ImuMPU6050::ImuMPU6050() {}

static void i2c0_mpu6050_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        float temperature;
        mpu6050_gyro_data_axes_t gyro_data;
        mpu6050_accel_data_axes_t accel_data;
        z_clock_t now;
        clock_gettime(CLOCK_REALTIME, &now);
        esp_err_t result = mpu6050_get_motion(dev_hdl, &gyro_data, &accel_data, &temperature);
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "device read failed (%s)", esp_err_to_name(result));
        }
        else
        {
            if (publisher_raw.topic.name != 0)
            {
                PicoRosso::set_timestamp(msg_raw.header.stamp, now);
                msg_raw.angular_velocity.x = gyro_data.x_axis;
                msg_raw.angular_velocity.y = gyro_data.y_axis;
                msg_raw.angular_velocity.z = gyro_data.z_axis;
                msg_raw.linear_acceleration.x = accel_data.x_axis;
                msg_raw.linear_acceleration.y = accel_data.y_axis;
                msg_raw.linear_acceleration.z = accel_data.z_axis;
                pr_publish(publisher_raw, msg_raw);
            }
            if (publisher_temperature.topic.name != 0 && msg_temperature.temperature != temperature)
            {
                PicoRosso::set_timestamp(msg_temperature.header.stamp, now);
                msg_temperature.temperature = temperature;
                pr_publish(publisher_temperature, msg_temperature);
            }
        }

        // pause the task per defined wait period
        vTaskDelayUntil(&last_wake_time, MPU6050_SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

bool ImuMPU6050::setup(i2c_master_bus_handle_t &i2c0_bus_hdl,
                       const char *topic_raw,
                       const char *topic_temp)
{
    ESP_LOGD(TAG, "Setting up...");

    // initialize i2c device configuration
    mpu6050_config_t dev_cfg = I2C_MPU6050_CONFIG;

    // init device
    mpu6050_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL)
    {
        ESP_LOGE(TAG, "handle init failed");
        return false;
    }

    if (topic_raw != NULL)
    {
        ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_raw);
        publisher_raw.topic.name = topic_raw;
        picoros_publisher_declare(&PicoRosso::node, &publisher_raw);
    }

    if (topic_temp != NULL)
    {
        ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_temp);
        publisher_temperature.topic.name = topic_temp;
        picoros_publisher_declare(&PicoRosso::node, &publisher_temperature);
    }

    /* create task pinned to the app core */
    xTaskCreatePinnedToCore(
        i2c0_mpu6050_task,
        I2C0_TASK_NAME,
        I2C0_TASK_STACK_SIZE,
        NULL,
        I2C0_TASK_PRIORITY,
        NULL,
        APP_CPU_NUM);

    ESP_LOGD(TAG, "Setting up done.");
    return true;
}
