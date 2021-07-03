# PATS

## Flashing the Anvil:

1. Start the bf_flash script to flash the firmware and betaflight settings:
```
cd ~/code/pats/config/firmwares/drone
./bf_flash.py -i <ID>
```
This may require to install `pip3 install pyserial` or in case of conda `conda install -c anaconda pyserial`. You'll also need `sudo apt install dfu-util`.
The script supports some command line arguments, like setting the drone id or selecting another settings file, see `./bf_flash.py --help`.

2. Download, unpack and start blheli-m: https://github.com/Asizon/blheli-configurator/releases/tag/1.2.0-jazzmaverick-beta4

3. Flash ESC drivers
Go to the `Flash all` option in BLHeli. Select file manually and go to `pats/config/firmwares/drone`.
Choose the driver file depending on the flightcontroller you are using.
for instance:
Flightcontroller Anvil = O-H-5 16.9
Flightcontroller Hammer CB v2 = S-H-50 16.9
Flightcontroller Hammer CB v2 = F-H-40 16.9

4. For the Anvil, apply the following settings, and then click write all settings:
![BLHeli settings](../../doc/BLHeliESCSettings_Anvil.png)


## Flashing the multimodule:
```
cd ~/code/pats/config/firmwares/multimodule
./flash.py
```
Same remarks as with drone flash script apply.

## Flashing realsense camera:
`sudo rs-fw-update -f ~/code/pats/config/firmwares/realsense/Signed_Image_UVC_5_12_14_50.bin`

## Initializing new Anvil charging pad:
### Programming Bootloader
1. Run Adruino IDE
2. Select Tools->Board->Arduino_Nano
3. Select Tools->Processor->Atmega328P
4. Place Olimex AVR-ISP-MK2 pins on the board
5. Select Tools->Burn_Bootloader 

### Flashing the CP2105 (only for Revision D)
1. Download AN721SW cp2105 costumizer (is available for Linux, but works best in Windows)
2. Run costumizer and change "3. SCI/ECI in GPIO Mode" to "0 - SCI/ECI in Modem Mode"
3. Program device

### Calibrating voltage measurement
1. Connect to charing pad and open in serial monitor
2. Take a charged drone and measure the voltage on the charging pins
3. Place the drone on the pad
4. Type "calib {voltage}" into the serial monitor (for instance "calib 4.23")
5. Serial monitor should now print correct voltage
6. To undo calibration type "reset"

## Flashing Anvil charging pad:

```
  cd $HOME/code/pats/config/firmwares/charging/avrdude  && ./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:$HOME/code/pats/config/firmwares/charging/charging_pad.ino.eightanaloginputs.hex:i
```

  Flash all connected charging pads:
```
  cd $HOME/code/pats/config/firmwares/charging/avrdude  &&  for i in {0..12}; do ./avrdude -C./avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB${i} -b115200 -D -Uflash:w:$HOME/code/pats/config/firmwares/charging/charging_pad.ino.eightanaloginputs.hex:i ; done
```

Sound decryption:

| Sounds            | Meaning                          |
|-------------------|----------------------------------|
| Mario theme       | Drone not detected               |
| Mario dies        | Drone removed                    |
| Level up          | Drone connected                  |
| Underground sound | Drone connected but no charging* |
| Low ticks         | Current is flowing               |
| Intermittent ticks| Needs calibration                |
| Startup beeps     | Encodes software version         /

 \* Contact resistance to the battery too high (Charging legs, broken wires, etc)
