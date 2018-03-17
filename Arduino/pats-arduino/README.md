# nrf24_cx10_pc

![Screenshot](https://raw.githubusercontent.com/perrytsao/nrf24_cx10_pc/master/arduino_cheerson_CX-10.JPG)
##nRF24L01 RC transmitter for Cheerson CX-10 with serial interface to a PC

This is a fork of [goebish/nrf24_multipro](https://github.com/goebish/nrf24_multipro) that has been modified so that it accepts input through a serial port instead of PPM signals from a transmitter. This code will always select Cheerson CX-10 Blue PCB protocol for operation, but the code for using other quadcopters is still intact, so it should be easy to modify for those who are interested.  

In order to use this code, you will need an Arduino, nrF24L01+ wireless cards, and a Cheerson CX-10, CX-10A, or CX-10C drone. This webpage has a [list of hardware needed](http://www.makehardware.com/2016/04/24/teach-your-pc-to-fly-a-mini-drone/). The Arduino and the wireless cards should be wired following this table:

| Arduino Uno    | NRF24L01+ Socket Adapter Board | Wire Color (in photo) |
|----------------|--------------------------------|-----------------------|
| GND            | GND                            | Orange                |
| 5V             | VCC                            | Red                   |
| D5 (Digital 5) | CE                             | Yellow                |
| A1 (Analog 1)  | CSN                            | Green                 |
| D4 (Digital 4) | SCK                            | Blue                  |
| D3 (Digital 3) | MO (MOSI)                      | Purple                |
| A0 (Analog 0)  | MI (MISO)                      | Gray                  |
| Not Used       | IRQ                            | White                 |

Although I haven't tested it yet, I expect that this code is compatible with the nrf24_multipro hardware that is used by goebish with his custom PCB.  

This sketch was tested with Arduino version 1.6.7.  nRF24_multipro.ino is the top-level code.  

The python script serial_test.py can be used to test the connection, although it's not really practical for flying.  

This code was developed as part of the [Teach your PC to Fly a Mini-Drone](http://www.makehardware.com/2016/04/24/teach-your-pc-to-fly-a-mini-drone/) project on [MakeHardware.com](www.makehardware.com).


