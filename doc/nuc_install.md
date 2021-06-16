## NUC installation instructions
1. Bios Power state:
    - Boot NUC with screen and keyboard
    - Press F2 during boot
    - Go to power tab, select Power state: always on
    - Go to the boot tab, disable secure boot
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
    - Select sda1 58.G_ntfs_MULTIBOOT…. (which is the usb dongle)
    - Select the image:
      - Select the “Images” folder, and press enter
      - Press tab twice to select Done! And then press Enter to select that image.
      - It’ll show the source to be the usb stick. Check the size is 58.4GB, press Enter.  
    - Select Beginner
    -  Select **“restoredisk”**
    - Check the name, select the image **with the most recent date!**
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
    - `sudo nano /etc/hosts`     Change `127.0.1.1 pats0` to reflect number on the label from step 3
    - Again for: `sudo nano /etc/hostname`
    - Reboot
6. Optional: connect wifi
7. Remove the `~/pats/flags/disable` file to enable the pats process in the background (preferably after the camera etc is connected)

All done!

## NUC create image instructions
0. Clean install
    - Download the Ubuntu 20.04.* lts *server* and put it on a usb stick
    - Download the following packages and also put it on the usb stick: 
        - libbluetooth3_5.53-0ubuntu3.2_amd64.deb
        - libc6_2.31-0ubuntu9_amd64.deb
        - libjansson4_2.12-1build1_amd64.deb
        - libmm-glib0_1.12.8-1_amd64.deb
        - libndp0_1.7-0ubuntu1_amd64.deb
        - libnl-route-3-200_3.4.0-1_amd64.deb
        - libnm0_1.22.10-1ubuntu1_amd64.deb
        - libpcsclite1_1.8.26-3_amd64.deb
        - libreadline8_8.0-4_amd64.deb
        - libteamdctl0_1.30-1_amd64.deb
        - network-manager_1.22.10-1ubuntu1_amd64.deb
        - wpasupplicant_2.9-1ubuntu4.3_amd64.deb
    - Download the newest version of the install script from github and put it on the stick
    - Arrange pats ssh files and put it on the stick
    - NUC 11:
        - Boot the NUC from the stick, select boot and install **with  hwe** and reboot
        - Enable GUC:
            - `sudo nano /etc/default/grub`    ----> `GRUB_CMDLINE_LINUX_DEFAULT="i915.enable_guc=2"`
            - `sudo update-grub`
    - NUC 7/8:
        - Boot the NUC from the stick, select boot and install and reboot
    - Install above packages (e.g. `mkdir -p ~/usb && sudo mount /dev/sda2 ~/usb -o rw` and cd to the folder with the debs and `sudo dpkg -i *`)
    - Enable WiFi (e.g. `sudo nmcli device wifi connect <SSID> password <PASS>`)
    - Run the install script
    - Reboot
    - Remotely open (via ssh) the base project with vscode, and install all extensions
    - Make sure both tunnels have been connected
2. Prep the image
    - **Make sure the system hostname is called pats0**
    - Set the disable flag
    - Delete all pats data, json and logs
3. Create the image with clonezilla
    - Connect NUC install the first usb stick
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
    - Select sda1 58.G_ntfs_MULTIBOOT…. (which is the usb dongle)
    - Select the image:
      - Select the “Images” folder, and press enter
      - Press tab twice to select Done! And then press Enter to select that image.
      - It’ll show the source to be the usb stick. Check the size is 58.4GB, press Enter.  
    - Select Beginner
    -  Select **“savedisk”**
    - Leave the default name, being the date of today for the image name.
    - Select the main hard drive **“nvmeOn1 500GB….”**
    - Select yes, to check the image
    - Select "fsck - interactively..." 
    - Select "yes check the saved image"
    - Select "senc - Not to encrypt ..."
    - Select poweroff
    - Follow the instructions and press y when asked
    - Wait 10 minutes until it powers off.
4. Copy the image:
    - Repeat the procedure for the other USB stick
    - Copy the image to the backups in BigPats

## Tools used to make the install stick:
1. YUMI
2. CloneZilla iso
3. Ubuntu 18.04.* LTS iso
4. Pats install script and ssh key files

