# PATS

## [Installation]
Start [install.sh](base/install/install.sh) with argument 0 (non monitoring mode) to install all requirements.

## [Firmwares](config/firmwares/README.md)

## [Simulator](./doc/simulator.md)

## Installing correct kernel:
`sudo apt install linux-headers-4.15.0-55 linux-headers-4.15.0-55-generic linux-image-4.15.0-55-generic linux-modules-4.15.0-55-generic linux-modules-extra-4.15.0-55-generic`  

`sudo nano /etc/default/grub`  
Change `GRUB_DEFAULT=0` to `GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 4.15.0-55-generic"`  
`sudo update-grub`  
Other known working kernel is 4.18.0.25

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
	Port 10001
```  

## valgrind
`valgrind --tool=memcheck --leak-check=full --track-origins=yes --track-fds=yes --suppressions=../../config/gst.supp --suppressions=/usr/local/share/opencv4/valgrind.supp --suppressions=/usr/local/share/opencv4/valgrind_3rdparty.supp --suppressions=/usr/share/glib-2.0/valgrind/glib.supp --gen-suppressions=all ./pats --generator 2>&1 | tee log.txt`
