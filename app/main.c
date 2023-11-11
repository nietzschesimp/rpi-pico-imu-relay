/**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2023, Tony Smith (@smittytone)
 * @version   1.4.2
 * @licence   MIT
 *
 */
#include "main.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"


/*
 * GLOBALS
 */

// FROM 1.0.1 Record references to the tasks
TaskHandle_t port_scan_task_handle = NULL;
TaskHandle_t console_task_handle = NULL;
TaskHandle_t sensor_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;

// Input character buffer
char g_inputBuffer[INPUT_BUFFER_SIZE];
int g_inputIdx = 0;

// Sensor struct
struct bno055_t sensor_handle;


/*
 * FUNCTIONS
 */

void input_char_callback(void* ignore) {

  log_debug("callback called!");
  int input = getchar_timeout_us(0);
  if (input < 0) {
    log_debug("Failed to fetch character");
    return;
  }
  printf("got character: %d\n", input);
  // Append data on buffer
  if ('\r' != input) {
    g_inputIdx = (g_inputIdx + 1) % INPUT_BUFFER_SIZE;
    g_inputBuffer[g_inputIdx] = (char)input;
    log_debug("saved character");
  }
  // Tell console task to process
  else {
    xTaskNotifyGive(console_task_handle);
    log_debug("processing line");
  }
}

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void led_task_pico(void* unused_arg) {

    // Set a delay time of exactly 1s
    const TickType_t ms_delay = 1000 / portTICK_PERIOD_MS;

    // Store the Pico LED state
    uint8_t pico_led_state = 0;
    
    // Configure the Pico's on-board LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    while (true) {
        // Turn Pico LED on
        log_debug("PICO LED FLASH");
        pico_led_state = 1;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        vTaskDelay(ms_delay);
        
        // Turn Pico LED off
        pico_led_state = 0;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        vTaskDelay(ms_delay);
    }
}

void sensor_task(void* unused_arg) {

  struct bno055_accel_t accel_xyz = {0, 0, 0};
  bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

  while(true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    bno055_read_accel_xyz(&accel_xyz);
    printf("Acceleration data:\n");
    printf("Ax: %d\n", accel_xyz.x);
    printf("Ay: %d\n", accel_xyz.y);
    printf("Az: %d\n", accel_xyz.z);
  }
}

/**
 * @brief Read messages from stdin and process the command.
 */
void console_task(void* unused_arg) {

  // Set callback to when a character is received
  stdio_set_chars_available_callback(input_char_callback, NULL);

  while (true) {
    // Wait until line is received
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(0 == strcmp("scan", g_inputBuffer)) {
      // Notify i2c scan task to run
      xTaskNotifyGive(port_scan_task_handle);
    }
  }
}

void i2c_port_scan(void* unused_arg) {

    while (true) {
      // Wait until task is notified to do a scan
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      printf("\nI2C Bus Scan\n");
      printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

      for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
          printf("%02x ", addr);
        }

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
          ret = -1;
        }
        else {
          ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
        }

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
      }
      printf("\nEnd Scan\n");
    }
}

s8 bno055_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {

  u8 array[cnt+1];
  array[0] = reg_addr;

  for (int i = 1; i < (cnt + 1); i++) {
    array[i] = reg_data[i-1];
  }

  int written = i2c_write_blocking(i2c_default, dev_addr, array, (cnt + 1), false);
  if (written != (cnt + 1)) {
    log_error("bno055_I2C_bus_write() - Failed to write message");
    return -1;
  }
  return BNO055_SUCCESS;
}


s8 bno055_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt) {

  int written = i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true);
  if (written < 1) {
    log_error("bno055_i2c_bus_read() - Failed to write request");
    return -1;
  }
  int read = i2c_read_blocking(i2c_default, dev_addr, reg_data, cnt, false);
  if (read != cnt) {
    log_error("bno055_i2c_bus_read() - Failed to read");
    return -1;
  }
  return BNO055_SUCCESS;
}


void bno055_i2c_bus_delay(u32 msec) {
  sleep_ms(msec);
}


void bno055_sensor_init(struct bno055_t* device) {

  device->bus_write = bno055_i2c_bus_write;
  device->bus_read = bno055_i2c_bus_read;
  device->delay_msec = bno055_i2c_bus_delay;
  device->dev_addr = BNO055_I2C_ADDR1;
}


/**
 * @brief Generate and print a debug message from a supplied string.
 *
 * @param msg: The base message to which `[DEBUG]` will be prefixed.
 */
void log_debug(const char* msg) {

#ifdef DEBUG
    uint msg_length = 9 + strlen(msg);
    char* sprintf_buffer = malloc(msg_length);
    sprintf(sprintf_buffer, "[DEBUG] %s\n", msg);
    printf("%s", sprintf_buffer);
    free(sprintf_buffer);
#endif
}

void log_error(const char* msg) {

  uint msg_length = 9 + strlen(msg);
  char* sprintf_buffer = malloc(msg_length);
  sprintf(sprintf_buffer, "[ERROR] %s\n", msg);
  printf("%s", sprintf_buffer);
  free(sprintf_buffer);
}

/**
 * @brief Show basic device info.
 */
void log_device_info(void) {

    printf("App: %s %s (%i)\n", APP_NAME, APP_VERSION, BUILD_NUM);
}


/*
 * RUNTIME START
 */
int main() {

    // Initialize the USB interface as stdio
    stdio_usb_init();
    while (!stdio_usb_connected()) {}

    // Initialize i2c
    i2c_init(i2c_default, 115200);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Initialize sensor
    bno055_sensor_init(&sensor_handle);
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
                                          "BNO055_ACCEL",
                                          512,
                                          NULL,
                                          4,
                                          &sensor_task_handle);

    BaseType_t pico_status = xTaskCreate(led_task_pico, 
                                         "PICO_LED_TASK", 
                                         128, 
                                         NULL, 
                                         3,
                                         &pico_task_handle);

    BaseType_t gpio_status = xTaskCreate(console_task,
                                         "CONSOLE_TASK",
                                         128, 
                                         NULL, 
                                         2,
                                         &console_task_handle);

    BaseType_t scan_status = xTaskCreate(i2c_port_scan,
                                         "I2C_PORT_SCAN",
                                         128,
                                         NULL,
                                         1,
                                         &port_scan_task_handle);
    

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (pico_status == pdPASS || gpio_status == pdPASS || scan_status == pdPASS || accel_status == pdPASS) {
        vTaskStartScheduler();
    }
    
FAIL_INIT:
    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
