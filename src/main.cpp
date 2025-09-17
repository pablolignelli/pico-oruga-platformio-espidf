#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include "driver/i2c_master.h"

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

// initialize master i2c 0 bus configuration
i2c_master_bus_config_t i2c0_bus_cfg = I2C0_MASTER_CONFIG_DEFAULT;
i2c_master_bus_handle_t i2c0_bus_hdl;

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define ZENOH_ROUTER_ADDRESS "tcp/192.168.10.235:7447"
#define ZENOH_NODE_NAME "oruga"

#include "wifi_connection.h"
WifiConnection wifi;

#include "picorosso.h"
PicoRosso picorosso;

#include "picorosso_bme680.h"
EnvBME680 env;

#include "mobility_skid.h"
MobilitySkid mobility;

#include "ticker.h"
Ticker ticker;

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

extern "C"
{
    void app_main(void);
}

void app_main()
{
    ESP_LOGI("main", "Booting...");

    // Start NVS and Wifi
    InitNVS();
    wifi.connect(WIFI_SSID, WIFI_PASSWORD);
    while (wifi.connected == false)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    // Start I2C
    /* instantiate i2c master bus 0 */
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_hdl));

    // PicoRosso initalization //////////////////////////////
    picorosso.setup(ZENOH_NODE_NAME, ZENOH_ROUTER_ADDRESS);

    // Modules initalization ////////////////////////////////
    ticker.setup("tick");
    env.setup(i2c0_bus_hdl);
    mobility.setup();

    // Publisher task
    // xTaskCreate(publish_twist, "publish_twist_task", 4096, NULL, 1, NULL);

    ESP_LOGI("main", "Booting completed.");

    while (true)
    {
        printf("1min!\n");
        // Wait for one second
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}