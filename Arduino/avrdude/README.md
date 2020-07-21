Flashing:

    ./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:$HOME/code/pats/Arduino/charger/charging_1_2/charging_pad.ino.eightanaloginputs.hex:i

Sound decryption:

| Sounds            | Meaning                          |
|-------------------|----------------------------------|
| Mario theme       | no drone detected                |
| Mario dies        | Drone removed                    |
| Level up          | Drone connected                  |
| Underground sound | Drone connected but no charging* |

 * Contact resistance to the battery too high (Charging legs, broken wires, etc)
