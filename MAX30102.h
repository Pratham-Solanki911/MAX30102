#ifndef MAX30102_H
#define MAX30102_H

#include <Wire.h>
#include "Filters.h"
#include <Arduino.h>

class MAX30102 {
public:
    MAX30102(uint8_t sdaPin = 21, uint8_t sclPin = 22);
    bool begin(uint8_t i2c_addr = MAX30102_ADDRESS, TwoWire *wire = &Wire, int32_t sensor_id = 30102);
    long readIR();
    float processSensorData(long irValue);
    bool isValidSignal(float value);
    void detectHeartRate(float maValue);
    long getBPM();
    long getAvgBPM();

private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool initializeSensor();

    uint8_t sda;
    uint8_t scl;
    uint8_t i2cAddress;
    TwoWire *i2cWire;
    int32_t sensorID;

    HighPassFilter hpFilter;
    LowPassFilter lpFilter;
    MovingAverageFilter<8> maFilter;

    static const uint8_t MAX30102_ADDRESS = 0x57;
    static const uint8_t REG_INTR_STATUS_1 = 0x00;
    static const uint8_t REG_INTR_ENABLE_1 = 0x02;
    static const uint8_t REG_FIFO_WR_PTR = 0x04;
    static const uint8_t REG_FIFO_RD_PTR = 0x06;
    static const uint8_t REG_FIFO_DATA = 0x07;
    static const uint8_t REG_MODE_CONFIG = 0x09;
    static const uint8_t REG_SPO2_CONFIG = 0x0A;
    static const uint8_t REG_LED1_PA = 0x0C;
    static const uint8_t REG_LED2_PA = 0x0D;

    long lastIrValue = 0;
    long lastBeatTime = 0;
    int beatCount = 0;
    long beatSum = 0;
    long beatAvg = 0;
    bool isPeak = false;
    long bpm = 0;
};

#endif // MAX30102_H
