#include "utility.h"
#include "Arduino.h"

float moving_average(float alpha, float value, float smoothed_value) {
    return alpha * value + (1.0f - alpha) * smoothed_value;
}

uint16_t serial_read_to_buf(unsigned char buf[MAX_PACKAGE_READ_SIZE]) {
    static uint16_t i = 0;
    for (i; i < MAX_PACKAGE_READ_SIZE;) {
        if (Serial.available() > 0) {
            buf[i] = Serial.read();
            if (buf[i] == '\n') {
                uint16_t ii = i;
                i = 0;
                return ii;
            }
            i++;
        } else
            return 0;
    }
    i = 0; // warning: overflow buf situation... should not happen
    return 0;
}
