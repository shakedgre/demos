#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "sensorReading.h"

void appMain() {
  DEBUG_PRINT("testing the sensor code...\n");

  if(Sensorbegin() == false) DEBUG_PRINT("AHT NOT DETECTED!\n");

  while (true){
    vTaskDelay(1000);
    float temp = getTemperature();
    vTaskDelay(100);
    float hum = getHumidity();
    DEBUG_PRINT("temp: %f, humidity: %f\n", (double)temp, (double)hum);
  }
  

}
