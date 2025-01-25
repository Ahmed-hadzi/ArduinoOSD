#include <SoftwareSerial.h>

#define VISTA_TX 10  // Arduino TX to Vista RX
#define VISTA_RX 11  // Arduino RX to Vista TX

SoftwareSerial vistaSerial(VISTA_RX, VISTA_TX);

void sendMSP(uint8_t cmd, const uint8_t *data, uint8_t dataSize) {
    uint8_t checksum = 0;
    vistaSerial.write('$');  // MSP header
    vistaSerial.write('M');
    vistaSerial.write('<');
    vistaSerial.write(dataSize);
    vistaSerial.write(cmd);
    checksum ^= dataSize;
    checksum ^= cmd;
    for (uint8_t i = 0; i < dataSize; i++) {
        vistaSerial.write(data[i]);
        checksum ^= data[i];
    }
    vistaSerial.write(checksum);
}

void sendBatteryData(float voltage, uint8_t batteryPercentage) {
    uint8_t analogData[3];
    analogData[0] = (uint8_t)(voltage * 10);  // Convert to 0.1V units
    analogData[1] = 0;  // mAh Drawn (not used)
    analogData[2] = batteryPercentage;  // Battery percentage

    sendMSP(109, analogData, sizeof(analogData));  // 109 = MSP_ANALOG
}

void setup() {
    vistaSerial.begin(115200);  // MSP protocol baud rate
}

void loop() {
    float batteryVoltage = 3.7 * 4;  // Example for 4S LiPo battery (14.8V)
    uint8_t batteryPercentage = (batteryVoltage - 14.0) * 100 / (16.8 - 14.0);

    sendBatteryData(batteryVoltage, batteryPercentage);

    delay(1000);  // Send data every second
}
