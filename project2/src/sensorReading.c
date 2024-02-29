#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "i2cdev.h"

#include "sensorReading.h"



void sensorInit(){
    DEBUG_PRINT("Initializing AHT20 2A...\n");
    i2cdevInit(I2C1_DEV);
    
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr, I2CDEV_NO_MEM_ADDR, (uint8_t)initCommand);
    xTaskCreate(readSensorData, 'AHT20', (2 * configMINIMAL_STACK_SIZE), NULL, 3, NULL);
    vTaskDelay(50);
}

void readSensorData(){

    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr, I2CDEV_NO_MEM_ADDR, (uint8_t)measCommand);
    uint64_t data = 0;
    uint8_t dataSection = 0;
    for(int i = 0; i < 7; i++){
        vTaskDelay(10);
        i2cdevRead(I2C1_DEV, (uint8_t)AHT20I2CAddr, 8, &dataSection);
        data = (data<<8) + dataSection;
    }
    uint32_t temperatureBytes = (data>>8)&0x00000000000FFFFF;
    uint32_t humidityBytes = (data>>28)&0x00000000000FFFFF;

    temperature = ((float)temperatureBytes / 1048576) * 200 - 50;
    humidity = ((float)humidityBytes / 1048576) * 100;
}