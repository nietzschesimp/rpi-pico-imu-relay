#include "bno055_driver_interface.h"


/**
 * \brief Interface funtion to write data to the BNO055 sensor
 * \param dev_addr  Address of the device
 * \param reg_addr  Address inside the sensor to read
 * \param red_data  Pointer to where to store the read data
 * \param cnt       Number of bytes to read
 * \return BNO055_SUCCESS if read data successfully, -1 otherwise
 */
s8 bno055_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt)
{
  u8 array[cnt+1];

  // Initialize fist byte with address to write
  array[0] = reg_addr;

  // Follow initial byte with desired data to write
  for (int i = 1; i < (cnt + 1); i++) {
    array[i] = reg_data[i-1];
  }

  // Write message to sensor
  int written = i2c_write_blocking(i2c_default, dev_addr, array, (cnt + 1), false);
  if (written != (cnt + 1)) {
    LOG_ERROR("bno055_I2C_bus_write() - Failed to write message");
    return -1;
  }
  return BNO055_SUCCESS;
}

/**
 * \brief Interface function to read data from the BNO055 function
 * \param dev_addr  Address of the device
 * \param reg_addr  Address inside the sensor to read
 * \param red_data  Pointer to where to store the read data
 * \param cnt       Number of bytes to read
 * \return BNO055_SUCCESS if read data successfully, -1 otherwise
 */
s8 bno055_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt)
{
  // Prime the sensor to move internal data pointer to the given address
  int written = i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true);
  if (written < 1) {
    LOG_ERROR("bno055_i2c_bus_read() - Failed to write request");
    return -1;
  }

  // Read the data coming from the sensor into the destination register
  int read = i2c_read_blocking(i2c_default, dev_addr, reg_data, cnt, false);
  if (read != cnt) {
    LOG_ERROR("bno055_i2c_bus_read() - Failed to read");
    return -1;
  }
  return BNO055_SUCCESS;
}


/**
 * \brief Function to cause a delay in our system to cause an intended delay
 * \param msec  Ammount of time in milliseconds to delay
 */
void bno055_i2c_bus_delay(u32 msec)
{
#if INCLUDE_vTaskDelay
  vTaskDelay(pdMS_TO_TICKS(msec));
#else
  sleep_ms(msec);
#endif
}


/**
 * \brief Interface function to initialize a bno055 handle.
 * \param device  Pointer to the device handle to initialize
 * \param addr    Address of the BNO055 sensor
 */
void bno055_sensor_init(struct bno055_t* device, u8 addr)
{
  // Initialize qwik connect i2c
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  i2c_init(i2c_default, 115200);

  // Populate device handle with perinent pointers and address
  device->bus_write = bno055_i2c_bus_write;
  device->bus_read = bno055_i2c_bus_read;
  device->delay_msec = bno055_i2c_bus_delay;
  device->dev_addr = addr;
}
