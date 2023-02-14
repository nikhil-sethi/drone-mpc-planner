#!/usr/bin/env bash

set -e

avrdude -v -patmega328p -carduino -P/dev/baseboard -b115200 -D -Uflash:w:./baseboard.ino.hex:i
