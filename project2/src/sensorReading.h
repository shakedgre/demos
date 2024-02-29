#ifndef sensorReading

#define sensorReading

#define AHT20I2CAddr 0x38
#define initCommand 0xE3 //0xBE
#define measCommand 0xAC
#define resetCommand 0xBA

#include <stdint.h>

float humidity;
float temperature;

void sensorInit();
void readSensorData();

#endif