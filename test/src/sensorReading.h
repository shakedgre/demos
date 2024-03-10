#ifndef sensorReading

#define sensorReading

#define AHT20I2CAddrWrite 0x70
#define AHT20I2CAddrRead 0x71

#define initCommand 0xE1 //0xBE
#define measCommand 0xAC
#define resetCommand 0xBA

#include <stdint.h>

float humidity;
float temperature;

void sensorInit();
void readSensorData();

#endif