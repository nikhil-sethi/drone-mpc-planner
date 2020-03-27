# Flash Multimodule
1. Install Arduino IDE from the website: https://www.arduino.cc/en/Main/Software.

 Don’t use the Arduino IDE from the Software Center! This caused problems in the past.

2. Follow the instructions here: https://github.com/pats-drones/DIY-Multiprotocol-TX-Module/blob/master/docs/Arduino_IDE_Boards.md to install board definitions in the Arduino IDE.

3. Get the firmware:
		git clone git@github.com:pats-drones/DIY-Multiprotocol-TX-Module.git
		git checkout pats-usb

4. Ensure correct user rights and udev rules following the instructions here: https://github.com/pats-drones/DIY-Multiprotocol-TX-Module/blob/master/docs/Compiling_STM32.md#upload-via-usb (see Linux section)[call comands from the repo `DIY-Multiprotocol-TX-Module`]

 Afterwards: Logout+Login user/ Reboot

5. Find the port of the multimodule with:
		ls /dev/ttyACM*
Which device appears/disappears if you plug/unplug the multimodule? (Default: /dev/ttyACM0)

6. If the multimodule already was flashed with the Pats code, you may need to open the port once first. This will allow the Arduino bootloader to start. E.g. start a screen session with:
		screen /dev/ttyACM0

 (and be sure to kill it with ctrl+a,k,y)

7. In the Arduino IDE open `DIY-Multiprotocol-TX-Module/Multiprotocol/Multiprotocol.ino`

8. Configure Arduino: https://github.com/pats-drones/DIY-Multiprotocol-TX-Module/blob/master/docs/Compiling_STM32.md#configure-the-arduino-ide
Instead of selecting the Upload method, set Tools -> Port: /dev/ttyACM*.....

9. Flash the multimodule!
