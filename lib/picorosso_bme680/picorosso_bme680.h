#ifndef __PICOROSSO_BME680_H
#define __PICOROSSO_BME680_H

#include "driver/i2c_master.h"

#define BME680_SAMPLE_INTERVAL_MS (1000)

// internal driver's debug level
#define BME680_DEBUG_LEVEL ESP_LOG_INFO 

#include "picoros.h"
#include "picoserdes.h"

class EnvBME680
{
public:
  EnvBME680();
  static bool setup(i2c_master_bus_handle_t &i2c0_bus_hdl,
                    const char *topic_temp = "internal/temperature",
                    const char *topic_humi = "internal/humidity",
                    const char *topic_pres = "internal/pressure",
                    const char *topic_gasr = "internal/gas_resistance");
};

#endif // __PICOROSSO_BME680_H
