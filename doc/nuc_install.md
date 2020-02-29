## NUC installation instructions
1. Bios Power state:
    - Boot NUC with screen and keyboard
    - Press F2 during boot
    - Go to power tab
    - Select Power state: always on
    - Save and shut down
2. Install image:
    - Connect NUC install usb stick
    - Power up and press F10 during boot
    - Select USB : SanDisk : Part 0 : Boot Drive
    - Select “System Tools”
    - Select “clonezilla-live- ...”
    - Select “clonezilla live  (Default settings….)”
    - Select English
    - Select Keep default keyboard layout
    - Select Start_Clonezilla
    - Select “device-image”
    - Select local_dev
    - Press Enter to detect usb devices
    - After the next screen appears, wait 5 seconds, press ctrl-C
    - Select sda1 58.G_ntfs_MULTIBOOT….
    - Select the image:
      - Select the “Images” folder, and press enter
      - Press tab twice to select Done! And then press Enter to select that image.
      - It’ll show the source to be the usb stick. Check the size is 58.4GB, press Enter.  
    - Select Beginner
    -  Select **“restoredisk”**
    - Check the name, select the image **"PATS-IMAGE-LATEST"**
    - Select the main hard drive **“nvmeOn1 500GB….”**
    - Select yes, to check the image
    - Select poweroff
    - Follow the instructions on the screen and press Enter when asked
    - Wait 2 minutes
    - It will give a warning message that you are about to destroy all data on the main drive… Say yes, (twice).
    - Wait two minutes until it powers off. (you can do step 3 already!)
3. Physically label the NUC
4. Remove USB dongle, boot up NUC
5. Change hostname:
    - `sudo nano /etc/hosts`     Change `127.0.1.1 pats-proto100` to reflect number on the label from step 3
    - Again for: `sudo nano /etc/hostname`
    - `sudo nano /etc/sudoers` and add `pats ALL = (root) NOPASSWD: /usr/sbin/rtcwake`
    - Reboot
6. Optional: update some random stuff.

All done!

## Tools used to make the install stick:
YUMI
# Contents:
CloneZilla iso
Ubuntu 18.04.1 LTS iso
Pats install script and ssh key files
