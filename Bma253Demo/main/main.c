#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Bma2x2.h"
#include "Bma253.h"

void app_main(void)
{
  Bma253_Init();
  
  while (1)
  {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}


