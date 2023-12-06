#ifndef BNO055_DRIVER_INTERFACE_H_
#define BNO055_DRIVER_INTERFACE_H_

// BNO055 Bosch driver
#include "bno055.h"

// FreeRTOS
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

// Pico SDK
#include <boards/adafruit_feather_rp2040.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "logging.h"

void bno055_sensor_init(struct bno055_t* device, u8 addr);

#endif