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
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
// C
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
// Pico SDK
#include "boards/adafruit_feather_rp2040.h"
#include "pico/stdlib.h"            // Includes `hardware_gpio.h`
#include "pico/binary_info.h"
#include "hardware/i2c.h"
// bno055
#include "bno055.h"


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
