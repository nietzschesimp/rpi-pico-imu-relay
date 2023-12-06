#include "bno055_driver_interface.h"


s8 bno055_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt)
{
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


s8 bno055_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt)
{
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


void bno055_i2c_bus_delay(u32 msec)
{
#if INCLUDE_vTaskDelay
  vTaskDelay(pdMS_TO_TICKS(msec));
#else
  sleep_ms(msec);
#endif
}


void bno055_sensor_init(struct bno055_t* device, u8 addr)
{
  // Initialize qwik connect i2c
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  i2c_init(i2c_default, 115200);

  device->bus_write = bno055_i2c_bus_write;
  device->bus_read = bno055_i2c_bus_read;
  device->delay_msec = bno055_i2c_bus_delay;
  device->dev_addr = addr;
}
