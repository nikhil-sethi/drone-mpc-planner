Nuc assembly (time: 3 min)
	1 RAM strip
	1 SSD
	
Change boot settings (Boot F8)
	Power - Secondary Power Settings - After Power Failure: Power On
	Boot - disable Secure Boot

Flash Nuc with flash USB
	
Nuc sign up to database
	pats-c.com/sys-adm
	
Serial number registration
	"ssh dash" in server
	"nano id_db.txt"

Label Nuc

Assemble PATS-C

Mount PATS-C on test pole

Check processes in tab 'Bb' and 'PATS'
	"ssh patsXXX"
	check processes
	in /pats/flags remove disable and disable_baseboard

Commands:
cd	(change dir)
ls	(list items)
rm	(remove)
sudo halt -p (shut down)

After installing the software on the NUC, it can be inserted in the printed basestation together with the following components:


 - NUC
 - 19V Power adapter
 - Interconnect PCB + power cable for NUC
 - Realsense camera + USB-cable
 - 4G stick + USB-cable
 - Cooling fan in top cover (air should be blown out of the system, text/arrow on fan should point outwards)
 - Optional: Multimodule + USB-cable ( only for hunting systems)


