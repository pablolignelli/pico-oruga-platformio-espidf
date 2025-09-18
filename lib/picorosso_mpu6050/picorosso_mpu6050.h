#ifndef __PICOROSSO_MPU6050_H
#define __PICOROSSO_MPU6050_H

#include "driver/i2c_master.h"

#define MPU6050_SAMPLE_INTERVAL_MS (500)

#include "picoros.h"
#include "picoserdes.h"

// TODO Set alarm limits to trigger out of sample interval

class ImuMPU6050
{
public:
  ImuMPU6050();
  static bool setup(i2c_master_bus_handle_t &i2c0_bus_hdl,
                    const char *topic_raw = "/imu/raw",
                    const char *topic_temp = "/imu/temperature");
};

#endif // __PICOROSSO_MPU6050_H
