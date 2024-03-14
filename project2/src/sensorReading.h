#ifndef sensorReading

#define sensorReading

#define AHT20I2CAddr_def 0x38

#define initCommand 0xBE //0xE1
#define measCommand 0xAC
#define resetCommand 0xBA

#include <stdint.h>

uint32_t humidity;
uint32_t temperature;

bool sensorQueriedTemperature = true;
bool sensorQueriedHumidity = true;

bool isConnected();
bool isCalibrated();
bool initialize();
void triggerMeasurement();
bool isBusy();



void sensorInit();
void readSensorData();

#endif