/**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2023, Tony Smith (@smittytone)
 * @version   1.4.2
 * @licence   MIT
 *
 */
#include "main.h"
#include "logging.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define I2C_TARGET_ADDR 0x69
#define I2C_RPI_SDA_PIN 0
#define I2C_RPI_SCL_PIN 1

TaskHandle_t rpi_encode_task_handle = NULL;
TaskHandle_t sensor_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;

// Sensor struct
struct bno055_t sensor_handle;

// I2C interface mutex
volatile QueueHandle_t sensor_queue = NULL;

static struct {
  char len;
  bool read_len;
  char index;
  bool index_written;
  char buffer[256];
} json_measurement;


/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void led_task_pico(void* unused_arg)
{
  // Store the Pico LED state
  uint8_t pico_led_state = 0;

  // Configure the Pico's on-board LED
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  while (true) {
    // Turn Pico LED on
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Turn Pico LED off
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void sensor_task(void* unused_arg)
{
  BNO055_RETURN_FUNCTION_TYPE rc;
  struct bno055_euler_double_t euler_hrp = {0.0, 0.0, 0.0};
  log_debug("Initializing BNO055 sensor configurations");

  // Set the operation to be an IMU
  bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
  log_debug("Initialized sensor configurations!");

  while(true) {
    vTaskDelay(pdMS_TO_TICKS(500));

    rc = bno055_convert_double_euler_hpr_deg(&euler_hrp);
    if (BNO055_SUCCESS == rc) {
      printf("Euler data:\n");
      printf("y: %f\n", euler_hrp.h);   // Print heading (yaw)
      printf("p: %f\n", euler_hrp.p);   // Print pitch
      printf("r: %f\n", euler_hrp.r);   // Print roll

      // Send data to be encoded
      xQueueSendToBack(sensor_queue, &euler_hrp, portMAX_DELAY);
    }
    else {
      log_error("Failed to read sensor data!");
    }
  }
}


void rpi_encode_task(void* unused_arg)
{
  struct bno055_euler_double_t msg = {0};
  char buf[128] = {0};
  int msg_len = 0;
  int count = 0;
  BaseType_t rc;

  while (true) {
    // Wait until a sensor measurement is received
    rc = xQueueReceive(sensor_queue, &msg, portMAX_DELAY);
    if (rc != pdPASS) {
      // Didn't sucessfully receive a message from the queue, skip
      log_error("Failed to receive data from sensor task!");
      continue;
    }

    if (json_measurement.len != 0) {
      // Data still hasnt been sent, wait until RPI finishes reading
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      log_debug("Reset index and len");
    }

    // Take message contents and render them to a JSON object
    snprintf(json_measurement.buffer,
             sizeof(json_measurement.buffer),
             "{"
             "yaw: %f,"
             "pitch: %f,"
             "roll: %f"
             "}\n",
             msg.h, msg.p, msg.r);
    json_measurement.len = strlen(json_measurement.buffer);
  }
}


static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
  switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
      if (!json_measurement.index_written) {
        // writes always start with the memory address
        json_measurement.index = i2c_read_byte_raw(i2c);
        json_measurement.index_written = true;
        log_debug("Set index");
      }
      break;
    case I2C_SLAVE_REQUEST: // master is requesting data
    {
      // If no bytes to read
      if (json_measurement.len == 0) {
        log_debug("No bytes to read!");
        break;
      }

      // If RPI hasn't read the length, give it the length
      if (!json_measurement.read_len) {
        i2c_write_byte_raw(i2c, json_measurement.len);
        json_measurement.read_len = true;
        json_measurement.index = 0;
        log_debug("Read length");
      }
      else {
        i2c_write_byte_raw(i2c, json_measurement.buffer[json_measurement.index]);
        json_measurement.index++;
        json_measurement.len--;
      }
      break;
    }
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
    {
      log_debug("Finished reading");
      json_measurement.index_written = false;
      if (json_measurement.len == 0) {
        log_debug("Notifying encoder to continue");
        json_measurement.read_len = false;
        json_measurement.index = 0;
        xTaskNotifyGive(rpi_encode_task_handle);
      }
      break;
    }
    default:
    {
      log_error("Got unexpected event from i2c interface!");
      break;
    }
  }
}

/**
 * @brief Show basic device info.
 */
void log_device_info(void)
{
  printf("App: %s %s (%i)\n", APP_NAME, APP_VERSION, BUILD_NUM);
}


/*
 * RUNTIME START
 */
int main()
{
    // Set clock rate
    bool rc = set_sys_clock_khz(configCPU_CLOCK_HZ/1000, true);
    if (!rc) {
      goto FAIL_INIT;
    }
    //setup_default_uart();

    // Initialize stdio
    stdio_usb_init();

    // Initialize secondary i2c interface
    gpio_init(I2C_RPI_SDA_PIN);
    gpio_set_function(I2C_RPI_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_RPI_SDA_PIN);
    gpio_init(I2C_RPI_SCL_PIN);
    gpio_set_function(I2C_RPI_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_RPI_SCL_PIN);
    i2c_slave_init(i2c0, I2C_TARGET_ADDR, &i2c_slave_handler);

    // Initialize sensor
    bno055_sensor_init(&sensor_handle, BNO055_I2C_ADDR1);
    s8 sensor_rc = bno055_init(&sensor_handle);
    if (sensor_rc != 0) {
      log_error("Failed to initalize sensor!");
        goto FAIL_INIT;
    }
    sensor_rc = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if (sensor_rc != 0) {
      log_error("Failed to set power mode of sensor!");
      goto FAIL_INIT;
    }

    // Log app info
    log_device_info();
    
    BaseType_t accel_status = xTaskCreate(sensor_task,
                                          "BNO055_READ_TASK",
                                          4096,
                                          NULL,
                                          4,
                                          &sensor_task_handle);

    BaseType_t pico_status = xTaskCreate(led_task_pico, 
                                         "PICO_LED_TASK", 
                                         4096,
                                         NULL, 
                                         3,
                                         &pico_task_handle);

    BaseType_t rpi_status = xTaskCreate(rpi_encode_task,
                                        "RPI_ENCODE_TASK",
                                        4096,
                                        NULL,
                                        2,
                                        &rpi_encode_task_handle);

    sensor_queue = xQueueCreate(16, sizeof(struct bno055_euler_double_t));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (pico_status == pdPASS ||
        accel_status == pdPASS ||
        rpi_status == pdPASS)
    {
        vTaskStartScheduler();
    }
    
FAIL_INIT:
    // We should never get here, but just in case...
    while(true) {
      for (int ii = 0; ii < 3; ii++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
      }
      sleep_ms(400);
    }
}
