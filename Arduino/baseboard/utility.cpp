#include "utility.h"
#include "Arduino.h"

float moving_average(float alpha, float value, float smoothed_value) {
    return alpha * value + (1.0f - alpha) * smoothed_value;
}

bool serial_read_pkg(unsigned char buf[MAX_PACKAGE_READ_SIZE]) {
    static unsigned char read_state = 0;
    static int  i = 0;
    static unsigned char len = 0;
    while (Serial.available()) {
        buf[i] = Serial.read();

        switch (read_state) {
            case 0: {
                    if (buf[i] == BASEBOARD_PACKAGE_PRE_HEADER)
                        read_state++;
                    else
                        i = -1;
                    break;
            } case 1: {
                    read_state++;
                    break;
            }  case 2: {
                    if (static_cast<uint16_t>(buf[i - 1]) == FIRMWARE_VERSION)
                        read_state++;
                    else {
                        read_state = 0;
                        i = -1;
                    }
                    break;
            } case 3: {
                    read_state++;
                    switch (buf[i]) {
                        case header_SerialNUC2BaseboardChargingPackage:
                            len = sizeof(SerialNUC2BaseboardChargingPackage);
                            break;
                        case header_SerialNUC2BaseboardLedPowerPackage:
                            len = sizeof(SerialNUC2BaseboardLedPowerPackage);
                            break;
                        case header_SerialNUC2BaseboardWatchdogPackage:
                            len = sizeof(SerialNUC2BaseboardWatchdogPackage);
                            break;
                        case header_SerialNUC2BaseboardFanPackage:
                            len = sizeof(SerialNUC2BaseboardFanPackage);
                            break;
                        case header_SerialNUC2BaseboardNUCResetPackage:
                            len = sizeof(SerialNUC2BaseboardNUCResetPackage);
                            break;
                        case header_SerialNUC2BaseboardEEPROMPackage:
                            len = sizeof(SerialNUC2BaseboardEEPROMPackage);
                            break;
                        case header_SerialNUC2BaseboardRGBLEDPackage:
                            len = sizeof(SerialNUC2BaseboardRGBLEDPackage);
                            break;
                        case header_SerialExecutor2BaseboardAllowChargingPackage:
                            len = sizeof(SerialExecutor2BaseboardAllowChargingPackage);
                            break;
                        default:
                            len = 0;
                            read_state = 0;
                            i = -1;
                            break;
                    }
                    break;
            } case 4: {
                    if (i - 1 == len && buf[i ] == '\n') {
                        read_state = 0;
                        i = -1;
                        return true;
                    } else if (i - 1 >= len) {
                        read_state = 0;
                        i = -1;
                    }
                    break;
            } default:
                break;
        }
        i = (i + 1) % MAX_PACKAGE_READ_SIZE;
    }
    return false;
}
