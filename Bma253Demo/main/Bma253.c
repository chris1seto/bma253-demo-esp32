#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "Bma2x2.h"

#define BMA253_PERIOD       (20 / portTICK_PERIOD_MS)
#define I2C_TIMEOUT         (5000 / portTICK_PERIOD_MS)

#define BMA253_I2C_FREQ_KHZ   100

#define INTERRUPTS_INT1 (1 << 0)
#define INTERRUPTS_INT2 (1 << 1)

static struct bma2x2_t bma2x2;

static EventGroupHandle_t bma253_interrupts;

static void Bma235Task(void* args);
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BMA2x2_delay_msek(u32 msek);

static const char* TAG = "BMA253";

#define ESP_INTR_FLAG_DEFAULT 0

static void gpio_isr_handler(void* arg);

void Bma253_Init(void)
{
	i2c_config_t conf;
  gpio_config_t io_conf;
  uint8_t i;
  uint8_t reg_content;

  bma253_interrupts = xEventGroupCreate();

  // !INT1
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pin_bit_mask = 1 << GPIO_NUM_18;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 0;
  io_conf.pull_down_en = 1;
  gpio_config(&io_conf);

  // !INT2
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pin_bit_mask = 1 << GPIO_NUM_19;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 0;
  io_conf.pull_down_en = 1;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_NUM_18, gpio_isr_handler, (void*)GPIO_NUM_18);
  gpio_isr_handler_add(GPIO_NUM_19, gpio_isr_handler, (void*)GPIO_NUM_19);

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
  bma2x2_set_power_mode(BMA2x2_MODE_LOWPOWER1);

  // Set bandwidth to highest possible
  bma2x2_set_bw(BMA2x2_ACCEL_BW_1000HZ_RANGE);

  bma2x2_set_intr_output_type(BMA2x2_INTR1_OUTPUT, PUSS_PULL);
  bma2x2_set_intr_output_type(BMA2x2_INTR2_OUTPUT, PUSS_PULL);
  bma2x2_set_intr_level(BMA2x2_INTR1_LEVEL, ACTIVE_HIGH);
  bma2x2_set_intr_level(BMA2x2_INTR2_LEVEL, ACTIVE_HIGH);

  //bma2x2_set_source(BMA2x2_SOURCE_TAP, INTR_ENABLE);

  //bma2x2_set_orient_enable(1);

  //bma2x2_set_intr_enable(BMA2x2_FLAT_INTR, INTR_ENABLE);
  //bma2x2_set_intr_orient(BMA2x2_INTR1_ORIENT, INTR_ENABLE);

  // Enable interrupts
  bma2x2_set_intr_enable(BMA2x2_SINGLE_TAP_INTR, INTR_ENABLE);
  //bma2x2_set_intr_enable(BMA2x2_DOUBLE_TAP_INTR, INTR_ENABLE);
  bma2x2_set_intr_single_tap(BMA2x2_INTR1_SINGLE_TAP, INTR_ENABLE);
  //bma2x2_set_intr_double_tap(BMA2x2_INTR2_DOUBLE_TAP, INTR_ENABLE);

  bma2x2_set_latch_intr(BMA2x2_LATCH_DURN_500US);

  for (i = 0; i < 0x3f; i++)
  {
    bma2x2.bus_read(bma2x2.dev_addr, i, &reg_content, 1);
    printf("0x%x = %x\r\n", i, reg_content);
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Start the Bma253 task
  xTaskCreatePinnedToCore(Bma235Task, TAG, 3072, NULL, 8, NULL, 0);
}

static void gpio_isr_handler(void* arg)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t gpio_num = (uint32_t)arg;

  switch (gpio_num)
  {
    // INT1
    case GPIO_NUM_18:
      xEventGroupSetBitsFromISR(bma253_interrupts, INTERRUPTS_INT1, &xHigherPriorityTaskWoken);
      break;

    // INT2
    case GPIO_NUM_19:
      xEventGroupSetBitsFromISR(bma253_interrupts, INTERRUPTS_INT2, &xHigherPriorityTaskWoken);
      break;

    default:
      break;
  }
}

s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  i2c_cmd_handle_t cmd;

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

  cmd = i2c_cmd_link_create();

  // Send start condition
  i2c_master_start(cmd);

  // Write address
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

  // Write reg addr
  i2c_master_write_byte(cmd, reg_addr, true);

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
  EventBits_t set_bits;
  uint8_t int_status;

  while (true)
  {
    set_bits = xEventGroupGetBits(bma253_interrupts);

    // Int 1 is set
    if (set_bits & INTERRUPTS_INT1)
    {
      xEventGroupClearBits(bma253_interrupts, INTERRUPTS_INT1);
      printf("INT1!!!!!\r\n");
    }

    // Int 2 is set
    if (set_bits & INTERRUPTS_INT2)
    {
      xEventGroupClearBits(bma253_interrupts, INTERRUPTS_INT2);
      printf("INT2!!!!!\r\n");
    }

    // Read Xyz
    bma2x2_read_accel_xyz(&xyz);
    bma2x2_get_intr_stat(&int_status);

    printf("%i, %i, %i, %i\r\n", xyz.x, xyz.y, xyz.z, int_status);

    vTaskDelay(BMA253_PERIOD);
  }
}