# PATS

## Flashing Betaflight:

Start the bf_flash script to flash the firmware and betaflight settings:
```
cd ~/code/pats/config/
./bf_flash.sh
```

Apply the following settings to BLHeli:  
1. Set `Beacon Delay` to `infinite`
2. Set `Beep Strenght` to 255
3. Set startup power to max (1.5)
4. Disable programming by tx check

Old manual way:

1. Put drone in bootloader mode. E.g. type `bl` in cli in betaflight configurator.  
2. Flash: `dfu-util -s 0x08000000 -a 0 -D ~/code/pats/config/trashcan_firmware.bin` 
3. In betaflight configurator, go to the CLI tab and paste https://github.com/pats-drones/pats/blob/master/config/BF_Trashcan.txt
4. `save`
5. Calibrate the drone. E.g. in the configurator, or by typing `c` in our program.

To retrieve firmware from the drone. (not needed normally)  
`dfu-util -s 0x08000000:413040 -a 0 -U firmware_dump.bin`  

## Flashing realsense camera:
`sudo rs-fw-update -f ~/code/pats/config/Signed_Image_UVC_5_12_3_0.bin`

## Installing correct kernel:
`sudo apt install linux-headers-4.15.0-55 linux-headers-4.15.0-55-generic linux-image-4.15.0-55-generic linux-modules-4.15.0-55-generic linux-modules-extra-4.15.0-55-generic`  

`sudo nano /etc/default/grub`  
Change `GRUB_DEFAULT=0` to `GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 4.15.0-55-generic"`  
`sudo update-grub`  
Other known working kernel is 4.18.0.25

## Converting videos
Convert first 10s videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -ss 00:00:00 -to 00:00:10 -c:v copy {}_first10s.mp4 \;` 

Convert videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v copy {}.mp4 \; -exec rm {} \;` 

Re-encode old videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v libx264 -level 3.0 -pix_fmt yuv420p -crf 21 -preset slow {}_reencoded.mp4 \; -exec rm {} \;` 

Re-encode color result video's (not necessary anymore):
`find -iname videoResult.avi -exec ffmpeg -i {} -c:v libx264 -preset slow -pix_fmt yuv420p -profile:v high -level 4.0 -b:v 15M -bf 2 -crf 18 {}.mp4 \;`

`find -iname videoResult.avi -exec ffmpeg -i {} -f matroska -c:v libvpx -crf 21 -b:v 15M {}.mkv \;` 


## Installing new keys:  
`ssh-keygen -t rsa -b 4096 -C "pats@pats.com"`  
`ssh-copy-id -i ~/.ssh/pats_id_rsa.pub patsX`  
To use the key on a new computer:  
`ssh-add ~/.ssh/pats_id_rsa`  
And add it to the ssh config, eg:  
```
Host pats1
	Hostname localhost
	IdentityFile ~/.ssh/pats_id_rsa
	User pats
	ProxyJump mavlab-gpu
	Port 6100
```  

