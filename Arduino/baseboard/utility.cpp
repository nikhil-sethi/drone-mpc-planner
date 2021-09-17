#include "utility.h"
#include "Arduino.h"

float moving_average(float alpha, float value, float smoothed_value) {
    return alpha * value + (1.0f - alpha) * smoothed_value;
}

bool serial_input(char *buf, int size) {
    static int i = 0;
    for (i; i < size;) {
        if (Serial.available() > 0) {
            buf[i] = Serial.read();
            if (buf[i] == '\n' || buf[i] == '\0') {
                buf[i] = '\0';
                i = 0;
                return true;
            }
            i++;
        }
        else {
            return false;
        }
    }
    i = 0;
    memset(buf, 0, size);
    return false;
}

bool match_command(char *input, const char *command) {
    int cmd_length = strlen(command);
    int inp_length = strlen(input);

    if (inp_length < cmd_length || cmd_length == 0)
        return false;
    else
        for (int i = 0; i < cmd_length; i++) {
            if (input[i] != command[i])
                return false;
        }

    return true;
}
