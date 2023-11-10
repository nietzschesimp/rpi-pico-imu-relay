/**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2023, Tony Smith (@smittytone)
 * @version   1.4.2
 * @licence   MIT
 *
 */
#include "main.h"


/*
 * GLOBALS
 */
// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Set a delay time of exactly 1s
const TickType_t ms_delay = 1000 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t gpio_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;
TaskHandle_t port_scan_task_handle = NULL;


/*
 * FUNCTIONS
 */

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
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        log_debug("PICO LED FLASH");
        pico_led_state = 1;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);
        
        // Turn Pico LED off an add the LED state
        // to the FreeRTOS xQUEUE
        pico_led_state = 0;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);
    }
}


/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void led_task_gpio(void* unused_arg) {

    // This variable will take a copy of the value
    // added to the FreeRTOS xQueue
    uint8_t passed_value_buffer = 0;
    
    // Configure the GPIO LED
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    
    while (true) {
        // Check for an item in the FreeRTOS xQueue
        if (xQueueReceive(queue, &passed_value_buffer, portMAX_DELAY) == pdPASS) {
            // Received a value so flash the GPIO LED accordingly
            // (NOT the sent value)
            if (passed_value_buffer) log_debug("GPIO LED FLASH");
            gpio_put(RED_LED_PIN, passed_value_buffer == 1 ? 0 : 1);
        }
    }
}

void i2c_port_scan(void* unused_arg) {

    vTaskDelay(pdMS_TO_TICKS(10000));

    // Initialize i2c port
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
      if (addr % 16 == 0) {
        printf("%02x ", addr);
      }

      // Perform a 1-byte dummy read from the probe address. If a slave
      // acknowledges this address, the function returns the number of bytes
      // transferred. If the address byte is ignored, the function returns
      // -1.

      // Skip over any reserved addresses.
      int ret;
      uint8_t rxdata;
      if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78)
        ret = -1;
      else
        ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

      printf(ret < 0 ? "." : "@");
      printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
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

    // Enable STDIO
#ifdef DEBUG
    stdio_usb_init();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);

    // Log app info
    log_device_info();
#endif
    
    // Set up two tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t pico_status = xTaskCreate(led_task_pico, 
                                         "PICO_LED_TASK", 
                                         128, 
                                         NULL, 
                                         3,
                                         &pico_task_handle);

    BaseType_t gpio_status = xTaskCreate(led_task_gpio, 
                                         "GPIO_LED_TASK", 
                                         128, 
                                         NULL, 
                                         2,
                                         &gpio_task_handle);

    BaseType_t scan_status = xTaskCreate(i2c_port_scan,
                                         "I2C_PORT_SCAN",
                                         128,
                                         NULL,
                                         1,
                                         &port_scan_task_handle);
    
    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (pico_status == pdPASS || gpio_status == pdPASS || scan_status == pdPASS) {
    // if (pico_status == pdPASS) {
        vTaskStartScheduler();
    }
    
    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
