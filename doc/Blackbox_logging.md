All files can be found in Code/pats/config/firmware/drone

# Info
Flight controller: IFF411 PRO ([link](https://shop.iflight-rc.com/index.php?route=product/product&path=20_27_132&product_id=848))  
Current pats firmware: `pats_BlackBox_4.2.0.bin`  
Settings file: `BF_Trashcan_Blackbox_4.2.0.txt`  
RC receiver: Frsky R-XSR with `RXSR_FCC_ACCST_191112.frk`  

# Coding & Flashing
In case of doing your own build, use the following betaflight building target: `make STM32F411` (instead of `make CRAZYBEEf4FR` which we use for all our other flightcontrollers)

# Logging
The drone will save all its info on flash which can be save and erased in the blackbox tab in Betaflight configurator. From there you can reset the drone to mass storage mode and use it like a usb stick. To use this log you'll need Betaflight blackbox explorer. ([link](https://github.com/betaflight/blackbox-log-viewer/releases))
In order to log rpm set the following setting in the cli: `set debug_mode =  DSHOT_RPM_TELEMETRY`

# Binding
There is a seperate rc receiver on this drone which cannot be controlled fully through software. So binding has to happen the old way. Find a toothpick, press the tiny button WHILE plugging in the battery. Binds only to D16 protocol!

# To build another one:
Solder an R-XSR to the IFF411 PRO in the SBUS configuration, see the image below and make sure to copy the setting just to be sure:

![Image of wiring](https://user-images.githubusercontent.com/41873016/83013935-1a5d3300-a01e-11ea-84e0-2b63d22602ee.png)

Flash the R-XSR with the correct firmware using the Xlite. Set the Xlite in D16 mode and start binding

# Additional info
https://github.com/pats-drones/pats/issues/300
