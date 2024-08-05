# MAX30102 Arduino Library

The `MAX30102` Arduino library provides an easy-to-use interface for the MAX30102 pulse oximeter and heart rate sensor. This library allows you to initialize the sensor, read IR values, process sensor data, and detect heart rates with customizable filtering.

## Features

- Initialize the MAX30102 sensor with customizable I2C address and wire object
- Read IR values from the sensor
- Process sensor data using high-pass, low-pass, and moving average filters
- Detect heart rate and calculate beats per minute (BPM)
- Retrieve average BPM

## Installation

1. Download the library from this repository.
2. Extract the downloaded ZIP file.
3. Copy the extracted folder to your Arduino libraries directory (`~/Documents/Arduino/libraries` on Windows and Linux, `~/Documents/Arduino/libraries` on macOS).
4. Rename the folder to `MAX30102` if necessary.
5. Restart the Arduino IDE if it was already open.

## Usage

### Basic Example

```cpp
#include <MAX30102.h>

MAX30102 max30102;

void setup() {
    Serial.begin(115200);
    if (max30102.begin()) {
        Serial.println("MAX30102 Initialized Successfully");
    } else {
        Serial.println("MAX30102 Initialization Failed");
    }
}

void loop() {
    long irValue = max30102.readIR();
    float processedValue = max30102.processSensorData(irValue);

    if (max30102.isValidSignal(processedValue)) {
        max30102.detectHeartRate(processedValue);
        long bpm = max30102.getBPM();
        long avgBpm = max30102.getAvgBPM();
        Serial.print("BPM=");
        Serial.print(bpm);
        Serial.print(", Avg BPM=");
        Serial.println(avgBpm);
    } else {
        Serial.println("Invalid IR Signal");
    }

    delay(100); // Adjust as needed
}
