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
    vTaskDelay(110);
    //i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, (uint8_t)AHT20I2CAddrWrite);
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, (uint8_t)initCommand);
    //xTaskCreate(readSensorData, 'AHT20', (2 * configMINIMAL_STACK_SIZE), NULL, 3, NULL);
    
}

void readSensorData(){
    vTaskDelay(120);
    //uint64_t msg = (measCommand<<16)+0x3300;
    //i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, ((uint8_t)AHT20I2CAddrWrite));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, ((uint8_t)measCommand));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x33));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddrWrite, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x00));
    //i2cdevWrite(I2C1_DEV,AHT20I2CAddrWrite,24,msg);
    vTaskDelay(80);
    uint64_t data = 0;
    uint8_t dataSection = 0;
    int err = 8;
    for(int i = 0; i < 8; i++){
        vTaskDelay(40);
        err -= i2cdevRead(I2C1_DEV, (uint8_t)AHT20I2CAddrRead, 8, &dataSection);
        data = (data<<8) + dataSection;
    }
    uint32_t temperatureBytes = (uint32_t)(data>>8)&0x00000000000FFFFF;
    uint32_t humidityBytes = (uint32_t)(data>>28)&0x00000000000FFFFF;

    temperature = ((float)temperatureBytes / 1048576) * 200 - 50;
    humidity = ((float)humidityBytes / 1048576) * 100;
    DEBUG_PRINT("temp: %f, bytes: %lx, data: %llx, err: %d\n",(double)temperature, temperatureBytes, data, err);
}


void appMain() {
  DEBUG_PRINT("testing sensor\n");
  vTaskDelay(M2T(200));
  sensorInit();
  while(1) {
    vTaskDelay(M2T(1000));
    readSensorData();
  }
}
