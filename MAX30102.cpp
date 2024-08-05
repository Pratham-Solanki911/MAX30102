#include "MAX30102.h"

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

MAX30102::MAX30102(uint8_t sdaPin, uint8_t sclPin)
    : sda(sdaPin), scl(sclPin), i2cAddress(MAX30102_ADDRESS), i2cWire(&Wire), sensorID(30102), hpFilter(0.5, 10.0), lpFilter(0.5, 10.0), maFilter() {}

bool MAX30102::begin(uint8_t i2c_addr, TwoWire *wire, int32_t sensor_id) {
    i2cAddress = i2c_addr;
    i2cWire = wire;
    sensorID = sensor_id;
    i2cWire->begin(sda, scl);
    return initializeSensor();
}

bool MAX30102::initializeSensor() {
    writeRegister(REG_MODE_CONFIG, 0x40);
    delay(100);

    writeRegister(REG_INTR_ENABLE_1, 0xC0);

    writeRegister(REG_FIFO_WR_PTR, 0x00);
    writeRegister(REG_FIFO_RD_PTR, 0x00);
    writeRegister(REG_FIFO_DATA, 0x00);

    writeRegister(REG_SPO2_CONFIG, 0x27);
    writeRegister(REG_LED1_PA, 0x7F);
    writeRegister(REG_LED2_PA, 0x7F);
    writeRegister(REG_MODE_CONFIG, 0x03);

    uint8_t status = readRegister(REG_INTR_STATUS_1);
    return status == 0x00;
}

void MAX30102::writeRegister(uint8_t reg, uint8_t value) {
    i2cWire->beginTransmission(i2cAddress);
    i2cWire->write(reg);
    i2cWire->write(value);
    i2cWire->endTransmission();
}

uint8_t MAX30102::readRegister(uint8_t reg) {
    i2cWire->beginTransmission(i2cAddress);
    i2cWire->write(reg);
    i2cWire->endTransmission(false);
    i2cWire->requestFrom(i2cAddress, 1);

    if (i2cWire->available()) {
        return i2cWire->read();
    }
    return 0;
}

long MAX30102::readIR() {
    i2cWire->beginTransmission(i2cAddress);
    i2cWire->write(REG_FIFO_DATA);
    i2cWire->endTransmission(false);
    i2cWire->requestFrom(i2cAddress, 6);

    if (i2cWire->available() == 6) {
        uint32_t red = 0;
        red |= (long)i2cWire->read() << 16;
        red |= (long)i2cWire->read() << 8;
        red |= (long)i2cWire->read();

        uint32_t ir = 0;
        ir |= (long)i2cWire->read() << 16;
        ir |= (long)i2cWire->read() << 8;
        ir |= (long)i2cWire->read();

        return ir;
    }
    return 0;
}

float MAX30102::processSensorData(long irValue) {
    float hpValue = hpFilter.process(irValue);
    float lpValue = lpFilter.process(hpValue);
    return maFilter.process(lpValue);
}

bool MAX30102::isValidSignal(float value) {
    // Adjust these values based on testing with cows
    return (value >= 500 && value <= 1000000);
}

void MAX30102::detectHeartRate(float maValue) {
    if (maValue > lastIrValue && !isPeak) {
        isPeak = true;
    } else if (maValue < lastIrValue && isPeak) {
        isPeak = false;
        long currentTime = millis();
        long deltaTime = currentTime - lastBeatTime;
        lastBeatTime = currentTime;

        if (deltaTime > 300 && deltaTime < 2000) {
            bpm = 60000 / deltaTime;
            beatSum += bpm;
            beatCount++;
            beatAvg = beatSum / beatCount;
        }
    }
    lastIrValue = maValue;
}

long MAX30102::getBPM() {
    return bpm;
}

long MAX30102::getAvgBPM() {
    return beatAvg;
}
