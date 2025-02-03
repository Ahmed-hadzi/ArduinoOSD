#include <SoftwareSerial.h>

#define OSD_TX 7  // Arduino TX to Vista RX
#define OSD_RX 8  // Arduino RX to Vista TX

SoftwareSerial vistaSerial(OSD_TX, OSD_RX);

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

/*void sendArmingStatus(bool armed) {
    uint8_t statusData[8] = {0};  // Initialize all bytes to 0

    statusData[2] = armed ? 1 : 0;  // Arming flag (1 = armed, 0 = disarmed)
    statusData[3] = 0;  // High byte of flight mode flags (not used)
    
    sendMSP(101, statusData, sizeof(statusData));  // MSP_STATUS message
}*/

void setup() {
    vistaSerial.begin(115200);  // MSP protocol baud rate
}

void loop() {
    float batteryVoltage = 3.7 * 4;  // Example for 4S LiPo battery (14.8V)
    uint8_t batteryPercentage = (batteryVoltage - 14.0) * 100 / (16.8 - 14.0);

    //bool armed = false;
    sendBatteryData(batteryVoltage, batteryPercentage);
    //sendArmingStatus(armed);

    delay(1000);  // Send data every second
}