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

bool WRequestFrom8(uint8_t deviceAddr, uint16_t size,uint8_t* data) {
    // Initialize I2C device
    bool status;

    // Read from the I2C device
    status = i2cdevRead(I2C1_DEV, deviceAddr, size, data);

    return (status == true);
}

/*bool WRequestFrom64(uint8_t deviceAddr, uint16_t size,uint64_t* data) {
    // Initialize I2C device
    bool status;

    // Read from the I2C device
    status = i2cdevRead(I2C1_DEV, deviceAddr, size, data);

    return (status == true);
}*/

bool Sensorbegin(){
    if(isConnected() == false) return false;

    vTaskDelay(40);

    if (isCalibrated() == false){
        //Send 0xBE0800
        initialize();

        //Immediately trigger a measurement. Send 0xAC3300
        triggerMeasurement();

        vTaskDelay(75); //Wait for measurement to complete

        uint8_t counter = 0;

        while (isBusy()){
            vTaskDelay(1);
            if (counter++ > 100)
                return (false); //Give up after 100ms
        }
        if (isCalibrated() == false) return (false);
    }
    if (isCalibrated() == false) return (false);

    sensorQueriedHumidity = true;
    sensorQueriedTemperature = true;
    return true;
}

bool isConnected(){
    if (i2cdevWriteBit(I2C1_DEV,(uint8_t)AHT20I2CAddr_def,I2CDEV_NO_MEM_ADDR,1,true)) return true;
    vTaskDelay(20);
    if (i2cdevWriteBit(I2C1_DEV,(uint8_t)AHT20I2CAddr_def,I2CDEV_NO_MEM_ADDR,1,true)) return true;
    return false;
}

uint8_t getStatus(){
    uint8_t data;
    if(WRequestFrom8(AHT20I2CAddr_def,(uint8_t)1, &data)) return data;
    return 0;

}

bool isCalibrated(){
    return (getStatus() & (1<<3));
}

bool isBusy(){
    return (getStatus() & (1<<7));
}

bool initialize(){
    /*i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)initCommand));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x08));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x00));
    if (i2cdevWriteBit(I2C1_DEV,(uint8_t)AHT20I2CAddr_def,I2CDEV_NO_MEM_ADDR,1,true)) return true;
    return false;*/
    uint8_t txBuffer[3] = {initCommand, 0x08, 0x00};

    // Perform a write operation to the device
    bool success = i2cdevWrite(I2C1_DEV, AHT20I2CAddr_def, sizeof(txBuffer),txBuffer);
    return success;
}

bool triggerMeasurement(){
    /*i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)measCommand));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x33));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x00));
    if (i2cdevWriteBit(I2C1_DEV,(uint8_t)AHT20I2CAddr_def,I2CDEV_NO_MEM_ADDR,1,true)) return true;
    return false;*/
    uint8_t txBuffer[3] = {measCommand, 0x33, 0x00};

    // Perform a write operation to the device
    bool success = i2cdevWrite(I2C1_DEV, AHT20I2CAddr_def, sizeof(txBuffer),txBuffer);
    return success;
}

void readData(){
    temperature = 0;
    humidity = 0;
    uint8_t data[7];
    uint8_t dataParts[7];

    if(WRequestFrom8(AHT20I2CAddr_def,(uint16_t)7,data) == false){
        DEBUG_PRINT("breaking early\n");
        return;
    }
    for (int i = 6; i >= 0; i--){
        dataParts[6-i] = data[6-i];
        DEBUG_PRINT("%x",dataParts[6-i]);
    }
    DEBUG_PRINT("\n");
    
    uint32_t incoming = 0;
    
    incoming |= (uint32_t)dataParts[1] << (8*2);
    incoming |= (uint32_t)dataParts[2] << (8*1);
    uint8_t midByte = dataParts[3];
    incoming |= (uint32_t)midByte;
    humidity = incoming >> 4;

    temperature = (uint32_t)midByte << (8*2);
    temperature |= (uint32_t)dataParts[4] << (8*1);
    temperature |= (uint32_t)dataParts[5] << (8*0);

    temperature = temperature & ~(0xFFF00000);

    sensorQueriedHumidity = false;
    sensorQueriedTemperature = false;
    

}

bool softReset(){
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, (uint8_t)resetCommand);
    if (i2cdevWriteBit(I2C1_DEV,(uint8_t)AHT20I2CAddr_def,I2CDEV_NO_MEM_ADDR,1,true)) return true;
    return false;
}

float getTemperature(){
    if (sensorQueriedTemperature == true){
        //We've got old data so trigger new measurement
        triggerMeasurement();

        vTaskDelay(75); //Wait for measurement to complete

        uint8_t counter = 0;
        while (isBusy())
        {
            vTaskDelay(1);
            if (counter++ > 100)
                return (false); //Give up after 100ms
        }

        readData();
    }
    float tempCelsius = ((float)temperature / 1048576) * 200 - 50;

    //Mark data as old
    sensorQueriedTemperature = true;

    return tempCelsius;
}

float getHumidity(){
    if (sensorQueriedHumidity == true){
        //We've got old data so trigger new measurement
        triggerMeasurement();

        vTaskDelay(75); //Wait for measurement to complete

        uint8_t counter = 0;
        while (isBusy())
        {
            vTaskDelay(1);
            if (counter++ > 100)
                return (false); //Give up after 100ms
        }

        readData();
    }
    float relHumidity = ((float)humidity / 1048576) * 100;

    //Mark data as old
    sensorQueriedTemperature = true;

    return relHumidity;
}






void sensorInit(){
    DEBUG_PRINT("Initializing AHT20 2A...\n");
    i2cdevInit(I2C1_DEV);
    vTaskDelay(110);
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, (uint8_t)initCommand);
    //xTaskCreate(readSensorData, 'AHT20', (2 * configMINIMAL_STACK_SIZE), NULL, 3, NULL);
    
}

void readSensorData(){
    vTaskDelay(120);
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)measCommand));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x33));
    i2cdevWriteByte(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, I2CDEV_NO_MEM_ADDR, ((uint8_t)0x00));
    uint64_t data = 0;
    uint8_t dataSection = 0;
    int err = 0;
    for(int i = 0; i < 7; i++){
        vTaskDelay(40);
        err += i2cdevRead(I2C1_DEV, (uint8_t)AHT20I2CAddr_def, 8, &dataSection);
        data = (data<<8) + dataSection;
    }
    uint32_t temperatureBytes = (uint32_t)(data>>8)&0x00000000000FFFFF;
    uint32_t humidityBytes = (uint32_t)(data>>28)&0x00000000000FFFFF;

    temperature = ((float)temperatureBytes / 1048576) * 200 - 50;
    humidity = ((float)humidityBytes / 1048576) * 100;
    DEBUG_PRINT("temp: %f, bytes: %lx, data: %lx, err: %d\n",(double)temperature, temperatureBytes, (unsigned long)data, err);
}