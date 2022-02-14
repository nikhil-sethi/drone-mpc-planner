# Building a PATS-C system


## 1. Nuc assembly (time: 3 min)
- 1 RAM strip
- 1 SSD

Ensure correct assembly of RAM module:

![IMG_20220209_111140](https://user-images.githubusercontent.com/14312271/153180176-b3d0ffe1-f0ed-4357-b9a8-eb36c5735425.jpg)

## 2. Change boot settings
Press F8 while booting

Power --> Secondary Power Settings --> After Power Failure: Power On

Boot --> disable Secure Boot

## 3. Flash Nuc with flash USB
While running continue to step 4 - 6.
	
## 4. Add nuc to database

Add nuc data & number to [pats-c.com/sys-adm](pats-c.com/sys-adm)
	
## 5. Serial number registration

```ssh dash``` 

ctrl + a then 5

```nano id_db.txt```

add serial number to the bottom of the file, followed by the next patsNUMBER.

Label the Nuc. Print "PATSXXX" twice (for nuc and basestation)

"ctrl + X" to close, save under the same name.

close dash with "ctrl + A" then "d".

## 6. Assemble PATS-C
Add following components to the 3D printed enclosure:

 - NUC
 - 19V Power adapter
 - Interconnect PCB + power cable for NUC
 - Realsense camera + USB-cable
 - Cooling fan in top cover (air should be blown out of the system, text/arrow on fan should point outwards)
 - PATS logo stickers
 - Optional: 4G stick + USB-cable
 - Optional: Multimodule + USB-cable ( only for hunting systems)

## 7. Test PATS-C

Mount PATC-C on test pole and connect to mains.


``` ssh patsXXX ```

Check processes in tab 'Bb' and 'PATS'

in /pats/flags remove disable and disable_baseboard

to close use "ctrl + A" then "d" _(not "ctrl + C"!)_



Commonly used commands:

```cd```	(change dir)

```ls```	(list items)

```rm```	(remove)

```sudo halt -p``` (shut down)
