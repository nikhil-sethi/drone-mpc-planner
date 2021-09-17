#!/bin/sh

cd $HOME/code/pats/config/firmwares/charging/avrdude  && ./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/baseboard -b115200 -D -Uflash:w:$HOME/code/pats/config/firmwares/baseboard/baseboard.ino.with_bootloader.hex:i && cd -


