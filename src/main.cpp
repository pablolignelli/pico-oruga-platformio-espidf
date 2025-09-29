#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_err.h>
#include "driver/i2c_master.h"
#if defined(WIFI_SSID)
#include <nvs_flash.h>
#endif
#include "driver/gpio.h"

#define LED_WAIT_PIN GPIO_NUM_5
#define LED_CONN_PIN GPIO_NUM_18
#define I2C0_MASTER_PORT I2C_NUM_0
#define I2C0_MASTER_SDA_IO GPIO_NUM_21
#define I2C0_MASTER_SCL_IO GPIO_NUM_22

#define I2C0_MASTER_CONFIG_DEFAULT {           \
    .i2c_port = I2C0_MASTER_PORT,              \
    .sda_io_num = I2C0_MASTER_SDA_IO,          \
    .scl_io_num = I2C0_MASTER_SCL_IO,          \
    .clk_source = I2C_CLK_SRC_DEFAULT,         \
    .glitch_ignore_cnt = 7,                    \
    .flags = {.enable_internal_pullup = true}, \
}

///////////////////////////////////////////////////////////////////////////
// initialize master i2c 0 bus configuration
i2c_master_bus_config_t i2c0_bus_cfg = I2C0_MASTER_CONFIG_DEFAULT;
i2c_master_bus_handle_t i2c0_bus_hdl;

// #define WIFI_SSID ""
#define WIFI_PASSWORD ""

// #define ZENOH_ROUTER_ADDRESS "tcp/192.168.10.235:7447"
// #define ZENOH_ROUTER_ADDRESS "tcp/192.168.101.2:7447"
// #define ZENOH_ROUTER_ADDRESS "serial/UART_0#baudrate=115200"
#define ZENOH_ROUTER_ADDRESS "serial/UART_0#baudrate=921600"

#define ZENOH_NODE_NAME "pico_oruga"

#define ROS_DOMAIN_ID 100
///////////////////////////////////////////////////////////////////////////

#if defined(WIFI_SSID)
#include "wifi_connection.h"
WifiConnection wifi;
#endif

#include "picorosso.h"
PicoRosso picorosso;

#include "picorosso_bme680.h"
EnvBME680 env;

#include "picorosso_mpu6050.h"
ImuMPU6050 imu;

#include "mobility_skid.h"
MobilitySkid mobility;

#include "ticker.h"
Ticker ticker;

#if defined(WIFI_SSID)
void InitNVS()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}
#endif

extern "C"
{
    void app_main(void);
}

void app_main()
{
    ESP_LOGI("main", "Booting...");

    gpio_config_t io_conf_wait = {
        .pin_bit_mask = (1ULL << LED_WAIT_PIN), // Select GPIO
        .mode = GPIO_MODE_OUTPUT,               // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE          // Disable interrupts
    };
    gpio_config(&io_conf_wait);
    gpio_set_level(LED_WAIT_PIN, 1);

    gpio_config_t io_conf_conn = {
        .pin_bit_mask = (1ULL << LED_CONN_PIN), // Select GPIO
        .mode = GPIO_MODE_OUTPUT,               // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE          // Disable interrupts
    };
    gpio_config(&io_conf_conn);
    gpio_set_level(LED_CONN_PIN, 0);

// Start NVS and Wifi
#if defined(WIFI_SSID)
    InitNVS();
    wifi.connect(WIFI_SSID, WIFI_PASSWORD);
    while (wifi.connected == false)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif

    // Start I2C
    /* instantiate i2c master bus 0 */
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_hdl));

    // PicoRosso initalization //////////////////////////////
    picorosso.setup(ZENOH_NODE_NAME, ZENOH_ROUTER_ADDRESS, ROS_DOMAIN_ID);
    gpio_set_level(LED_CONN_PIN, 1);

    // Modules initalization ////////////////////////////////
    ticker.setup("tick");
    env.setup(i2c0_bus_hdl);
    imu.setup(i2c0_bus_hdl);
    mobility.setup();

    // Publisher task
    // xTaskCreate(publish_twist, "publish_twist_task", 4096, NULL, 1, NULL);

    gpio_set_level(LED_WAIT_PIN, 0);
    ESP_LOGI("main", "Booting completed.");

    while (true)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        picorosso.rosout.out("Alive!", __FILE__, __func__, __LINE__, ROSLOG_INFO);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(60000));
    }
}