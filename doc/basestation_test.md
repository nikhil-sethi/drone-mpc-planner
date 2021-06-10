The following steps need to be followed to check whether the system is function propperly.

1. Connect the adaptor and check whether the LED on the interconnect is on and whether the fan is turning.
2. Check the light on the 4g-stick it should be not blinking and blue.
3. Ssh to the system and rm the disable flag from pats/flags
4. Make sure that the pats proces is running, by checking the pats screen.
    a. Is the camera connected?
    b. Does the roll angle fall between the bounds?
5. Let the system run for a couple of days. In the mean time you can do other things.
6. Add the system to the database and the sim to [jasper](https://kpn.jasperwireless.com/).
7. After the system has run for a couple days check whether the following processes wroked.
    a. Is there monitoring data in the pats/data/processed file?
    b. Do the data folders contain cutted moths? 
    c. Are the renders made?
    d. Did the systems send its JSONS to dash?
    e. Check the error files. Are there errors? Did the files rotate? Are they send to dash?
8. If everything is working the system is ready to be shipped or installed.
