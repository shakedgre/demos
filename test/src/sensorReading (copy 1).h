#ifndef sensorReading

#define sensorReading

#define AHT20I2CAddrWrite 0x38//01110001
#define AHT20I2CAddrRead 0x38

#define initCommand 0xBE //0xE1 10111110, 11100001
#define measCommand 0xAC
#define resetCommand 0xBA

#include <stdint.h>

float humidity;
float temperature;

void sensorInit();
void readSensorData();

#endif