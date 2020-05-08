bootloader1:
./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:$HOME/code/pats/Arduino/charger/charging_2_1/charging_2_1.hex:i 

bootloader2:
./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:$HOME/code/pats/Arduino/charger/charging_2_1/charging_2_1.hex:i
