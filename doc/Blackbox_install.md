All files can be found in Code/pats/config/firmware/drone
to set up the the IFF411 PRO:

Solder the R-XSR to the IFF411 PRO in the SBUS configuration, see the image below and make sure to copy the setting just to be sure:
image

Flash the R-XSR with the following firmware using the Xlite
RXSR_FCC_ACCST_191112.frk

Connect the Xlite to the R-XSR.

set the Xlite in d16 mode and start binding
Press the boot button on the R-XSR and hold it until you've plugged it in the battery
Flash the FC with the following bin file
betaflight_4.1.5_STM32F411.bin
Paste the betaflight setting of the IFF411 PRO in to the CLI and save.
BF_IFF411_PRO_4-1-5.txt
Now the drone will save all its info on flash which can be save and erased in the blackbox tab in Betaflight configurator. To use this log you'll need Betaflight blackbox explorer.
https://github.com/betaflight/blackbox-log-viewer/releases

Here you can look at the log and save it into a CSV

original commit:
https://github.com/pats-drones/pats/issues/300
