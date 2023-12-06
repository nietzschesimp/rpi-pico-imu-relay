/**
 * RP2040 FreeRTOS Template
 *
 * @copyright 2023, Tony Smith (@smittytone)
 * @version   1.4.2
 * @license   MIT
 *
 */
#ifndef MAIN_H
#define MAIN_H


// FreeRTOS
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <task.h>

// C
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// Pico SDK
#include "boards/adafruit_feather_rp2040.h"
#include "pico/binary_info.h"
#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>

// bno055
#include "bno055.h"

// Local includes
#include "bno055_driver_interface.h"
#include "logging.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * CONSTANTS
 */
#define         RED_LED_PIN           20
#define         INPUT_BUFFER_SIZE     256
#define         I2C_BUFFER_LEN        8
#define         BNO055_I2C_ADDR       0x28

/**
 * PROTOTYPES
 */
void led_task_pico(void* unused_arg);
void console_task(void* unused_arg);
void log_debug(const char* msg);
void log_error(const char* msg);
void log_device_info(void);


#ifdef __cplusplus
}           // extern "C"
#endif


#endif      // MAIN_H
