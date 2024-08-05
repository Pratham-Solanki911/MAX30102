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
