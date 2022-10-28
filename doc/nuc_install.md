## NUC installation instructions
1. Scan / copy the NUC serial code, and update id_db.txt on dash
2. Bios Power state:
    - Boot NUC with screen and keyboard
    - Press F2 during boot
    - Go to power tab, select Power state: always on
    - Go to the boot tab, disable secure boot
    - Save and shut down
3. Install image:
    - Connect NUC install usb stick
    - Power up and press F10 during boot
    - Select USB : UEFI: SanDisk : Partition 1
    - Choose PATS BURN (or wait, because it auto selects that)
    - After a minute, select the *correct image* for the *correct NUC*
    - Say y
    - Wait a few minutes until it powers off.
4. Physically label the NUC
5. Remove USB dongle, boot up NUC
6. If the config is not automatically downloaded from id_db.txt, change hostname:
    - `sudo nano /etc/hosts`     Change `127.0.1.1 pats0` to reflect number on the label from step 3
    - Again for: `sudo nano /etc/hostname`
    - Download the wireguard config from `dash:wireguard_config/peers/patsID` and put and rename it into `/etc/wireguard/wg0.conf`
    - Reboot
7. Remove the `~/pats/flags/disable` file to enable the pats process in the background (preferably after the camera etc is connected)
8. Idem for the `disable_baseboard` and `disable_charging` flags, if required

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
    - NUC 7/8/10:
        - Boot the NUC from the stick, select boot and install and reboot
    - Ubuntu 20.04 server install: https://askubuntu.com/questions/1269493/ubuntu-server-20-04-1-lts-not-all-disk-space-was-allocated-during-installation
        - `sudo vgdisplay`
        - `sudo lvextend -l +100%FREE /dev/mapper/ubuntu--vg-ubuntu--lv`
        - `sudo resize2fs /dev/mapper/ubuntu--vg-ubuntu--lv`
    - Install above packages (e.g. `mkdir -p ~/usb && sudo mount /dev/sda2 ~/usb -o rw` and cd to the folder with the debs and `sudo dpkg -i *`)
    - Enable WiFi (e.g. `sudo nmcli device wifi connect <SSID> password <PASS>`)
    - `sudo nano /etc/default/grub`    ----> `GRUB_CMDLINE_LINUX_DEFAULT="quiet nosplash fsck.repair=yes i915.enable_guc=2"`
    - `sudo update-grub`
    - Run the install script
    - Reboot
    - Remotely open (via ssh) the base project with vscode, and install all extensions
    - Make sure both tunnels have been connected
2. Prep the image
    - **Make sure the system hostname is called pats0**
    - **Update the image version number in `~/dependencies/image_version`**
    - Set the disable flag
    - Delete all pats data, json and logs
    - Clean up the `~/.bash_history'
    - Make sure the correct branch is selected.
    - Make sure the following files are removed:
        - `/home/pats/dependencies/hostname_set`
        - `/home/pats/dependencies/timezone_set` 
        - `/etc/wireguard/wg0.conf`
3. Create the image with clonezilla
    - Power up and press F10 during boot
    - Select USB : UEFI: SanDisk : Partition 1
    - Choose `PATS Create Image`
    - Give the image a name in the form: `NUC**_v*.*_DATE-img`. Except for the date, this **must** be similar to the contents in `~/pats/dependencies/image_version`
    - Wait a few minutes until it powers off.
4. Copy the image:
    - Repeat the procedure for the other USB stick
    - Copy the image to the backups in BigPats
5. Update the default burn image
    - Edit, commit, push `subl ~/code/pats/config/clonezilla/grub.cfg `
    - Copy to both sticks

## Tools used to make the install stick:
1. YUMI
2. CloneZilla iso


