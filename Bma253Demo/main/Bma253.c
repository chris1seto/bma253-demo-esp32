#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "Bma2x2.h"

#define BMA253_PERIOD       (20 / portTICK_PERIOD_MS)
#define I2C_TIMEOUT         (5000 / portTICK_PERIOD_MS)

#define BMA253_I2C_FREQ_KHZ   100

struct bma2x2_t bma2x2;

static void Bma235Task(void* args);
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BMA2x2_delay_msek(u32 msek);

static const char* TAG = "BMA253";

void Bma253_Init(void)
{
	i2c_config_t conf;

  // Configure I2c
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = GPIO_NUM_23;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = BMA253_I2C_FREQ_KHZ * 1000;

	i2c_param_config(I2C_NUM_1, &conf);
	i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);

  // Config Bma253 shim
 	bma2x2.bus_write = BMA2x2_I2C_bus_write;
	bma2x2.bus_read = BMA2x2_I2C_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;
	bma2x2.dev_addr = BMA2x2_I2C_ADDR1;

  // Init accel
  if (bma2x2_init(&bma2x2) != SUCCESS)
  {
    ESP_LOGE(TAG, "Init fail");
    return;
  }

  // Set power to normal mode
  bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

  // Set bandwidth to highest possible
  bma2x2_set_bw(BMA2x2_ACCEL_BW_1000HZ_RANGE);

  // Start the Bma253 task
  xTaskCreatePinnedToCore(Bma235Task, TAG, 3072, NULL, 8, NULL, 0);
}

s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  i2c_cmd_handle_t cmd;
  
  printf("Read %i\r\n", cnt);

  cmd = i2c_cmd_link_create();

  // Send start condition
  i2c_master_start(cmd);

  // Write address
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  
  // Write reg addr
  i2c_master_write_byte(cmd, reg_addr, true);
  
  // Send start condition
  i2c_master_start(cmd);
  
  // Write address
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

  // Read from device  
  i2c_master_read(cmd, reg_data, cnt, I2C_MASTER_LAST_NACK);

  // Send stop condition
  i2c_master_stop(cmd);

  // Perform the transaction
  if (i2c_master_cmd_begin(I2C_NUM_1, cmd, I2C_TIMEOUT) != ESP_OK)
  {
    i2c_cmd_link_delete(cmd);

    return ERROR;
  }

  i2c_cmd_link_delete(cmd);

  return SUCCESS;
}

s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  i2c_cmd_handle_t cmd;
  
  printf("Write %i\r\n", cnt);

  cmd = i2c_cmd_link_create();

  // Send start condition
  i2c_master_start(cmd);

  // Write address
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

  // Write data
  i2c_master_write(cmd, reg_data, cnt, true);

  // Send stop condition
  i2c_master_stop(cmd);

  // Perform the transaction
  if (i2c_master_cmd_begin(I2C_NUM_1, cmd, I2C_TIMEOUT) != ESP_OK)
  {
    i2c_cmd_link_delete(cmd);

    return ERROR;
  }

  i2c_cmd_link_delete(cmd);

  return SUCCESS;
}

void BMA2x2_delay_msek(u32 msek)
{
  vTaskDelay(msek / portTICK_PERIOD_MS);
}

static void Bma235Task(void* args)
{
  struct bma2x2_accel_data xyz;

  while (true)
  {
    bma2x2_read_accel_xyz(&xyz);
    
    printf("%i, %i, %i\r\n", xyz.z, xyz.y, xyz.z);

    vTaskDelay(BMA253_PERIOD);
  }
}