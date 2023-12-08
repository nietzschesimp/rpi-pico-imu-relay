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
TaskHandle_t rpi_read_task_handle = NULL;
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
  LOG_TRACE("Initializing BNO055 sensor configurations");
  char buf[128] = {0};

  // Set the operation to be an IMU
  bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
  LOG_TRACE("Initialized sensor configurations!");

  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    LOG_TRACE("Reading sensor data");

    rc = bno055_convert_double_euler_hpr_deg(&euler_hrp);
    if (BNO055_SUCCESS == rc) {
      printf("Euler data:\n");
      printf("y: %f\n", euler_hrp.h);   // Print heading (yaw)
      printf("p: %f\n", euler_hrp.p);   // Print pitch
      printf("r: %f\n", euler_hrp.r);   // Print roll

      // Send data to be encoded
      xQueueSendToBack(sensor_queue, &euler_hrp, portMAX_DELAY);
      LOG_TRACE("Sent data to be encoded");
    }
    else {
      LOG_ERROR("Failed to read sensor data!");
    }
  }
}


void rpi_encode_task(void* unused_arg)
{
  struct bno055_euler_double_t msg = {0};
  int msg_len = 0;
  int count = 0;
  BaseType_t rc;

  while (true) {
    // Wait until a sensor measurement is received
    rc = xQueueReceive(sensor_queue, &msg, portMAX_DELAY);
    if (rc != pdPASS) {
      // Didn't sucessfully receive a message from the queue, skip
      LOG_ERROR("Failed to receive data from sensor task!");
      continue;
    }

    // Clear encoding buffer
    memset(json_measurement.buffer, 0x00, sizeof(json_measurement.buffer));

    // Take message contents and render them to a JSON object
    snprintf(json_measurement.buffer,
             sizeof(json_measurement.buffer),
             "{"
             "\"yaw\":%0.4f,"
             "\"pitch\":%0.4f,"
             "\"roll\":%0.4f"
             "}",
             msg.h, msg.p, msg.r);
    json_measurement.len = strlen(json_measurement.buffer);
    json_measurement.read_len = false;
    LOG_TRACE("Encoded sensor data into JSON to send");
    LOG_DEBUG(json_measurement.buffer);
  }
}


void rpi_read_task(void* unused_args)
{
  size_t available, num_send;
  while(true) {
    // Wait until notified that RPI wants to read
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (json_measurement.len <= 0) {
      // Tell RPI no samples are available
      i2c_write_byte_raw(i2c0, 0);
      LOG_TRACE("No bytes to read, notifying sensor reader...");
      // Notify sensor read task to take a new measurement
      xTaskNotifyGive(sensor_task_handle);
      continue;
    }

    // If RPI hasn't read the length, give it the length
    if (!json_measurement.read_len) {
      i2c_write_byte_raw(i2c0, json_measurement.len);
      json_measurement.read_len = true;
      json_measurement.index = 0;
      LOG_TRACE("Read length");
    }
    // Write bytes from the encoded measurement buffer
    else {
      num_send = i2c_get_write_available(i2c0);
      if (num_send > json_measurement.len) {
        num_send = json_measurement.len;
      }
      i2c_write_raw_blocking(i2c0, json_measurement.buffer + json_measurement.index, num_send);
      json_measurement.index += num_send;
      json_measurement.len -= num_send;
      printf("Sent %d bytes to RPI\n", num_send);
    }
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
        printf("Set index: %d\n", json_measurement.index);
      }
      break;
    case I2C_SLAVE_REQUEST: // master is requesting data
    {
      xTaskNotifyGive(rpi_read_task_handle);
      break;
    }
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
    {
      json_measurement.index_written = false;
      break;
    }
    default:
    {
      LOG_ERROR("Got unexpected event from i2c interface!");
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
    while(!stdio_usb_connected()){};
    log_device_info();
    log_init();

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
      LOG_FATAL("Failed to initalize sensor!");
      goto FAIL_INIT;
    }
    sensor_rc = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if (sensor_rc != 0) {
      LOG_FATAL("Failed to set power mode of sensor!");
      goto FAIL_INIT;
    }
    
    BaseType_t rpi_read = xTaskCreate(rpi_read_task,
                                      "RPI_READ_TASK",
                                      2048,
                                      NULL,
                                      2,
                                      &rpi_read_task_handle);

    BaseType_t rpi_encode = xTaskCreate(rpi_encode_task,
                                        "RPI_ENCODE_TASK",
                                        2048,
                                        NULL,
                                        3,
                                        &rpi_encode_task_handle);

    BaseType_t accel_status = xTaskCreate(sensor_task,
                                          "BNO055_READ_TASK",
                                          2048,
                                          NULL,
                                          4,
                                          &sensor_task_handle);

    BaseType_t pico_status = xTaskCreate(led_task_pico, 
                                         "PICO_LED_TASK",
                                         2048,
                                         NULL,
                                         5,
                                         &pico_task_handle);

    sensor_queue = xQueueCreate(4, sizeof(struct bno055_euler_double_t));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (pico_status == pdPASS ||
        accel_status == pdPASS ||
        rpi_encode == pdPASS ||
        rpi_read == pdPASS)
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
