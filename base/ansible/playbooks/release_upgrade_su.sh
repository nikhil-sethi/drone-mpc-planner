#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive
export DEBIAN_PRIORITY=critical

ubuntu_str=$(lsb_release -a | grep Release)
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P1_FLAG=/home/pats/dependencies/release_upgraded_P1_20230405.done

if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi

if [ -f /home/pats/dependencies/image_version ]; then
	IMAGE_VERSION=$(cat /home/pats/dependencies/image_version)
	if [[ $IMAGE_VERSION == "NUC10+11 v3" ]]; then
		mkdir -p /home/pats/pats/updates
		chmod 755 /home/pats/pats/updates
		cp /home/pats/dependencies/grub_20 /etc/default/grub
		rm  /home/pats/dependencies/grub_20
		apt-mark hold grub-efi-amd64
		apt remove -y sendmail
		update-grub
		unattended-upgrade -d
		apt upgrade -y
		apt autoremove
		apt install -y avrdude gdisk
		apt install -y hostapd isc-dhcp-server rfkill
		/bin/su - pats -c "touch /home/pats/pats/flags/disable_trapeye && rm /home/pats/.ssh/known_hosts && /usr/bin/ssh-keyscan -t rsa github.com >> /home/pats/.ssh/known_hosts && cd /home/pats/pats/release && /usr/bin/git pull --rebase"
		touch $RELEASE_UPGRADE_FLAG
		echo Please reboot me.
		exit 0
	fi
fi

if [ ! -f $RELEASE_UPGRADE_P1_FLAG ]; then
	if [[ $ubuntu_str == *"18.04"* ]] ; then
		apt remove -y sendmail
		# sed -i 's/^\(.*OnlyOnACPower\s*=\s*\)"true";/\1"false";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\s*\/\/\s*Unattended-Upgrade::OnlyOnACPower\s*"true";/Unattended-Upgrade::OnlyOnACPower "true";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\(.*Skip-Updates-On-Metered-Connections\s*=\s*\)"true";/\1"false";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\s*\/\/\s*Unattended-Upgrade::Skip-Updates-On-Metered-Connections\s*"true";/Unattended-Upgrade::Skip-Updates-On-Metered-Connections "true";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		unattended-upgrade -d | true
        apt upgrade -y
        rm /home/pats/dependencies/grub_20
        mkdir -p /home/pats/dependencies
		mv /usr/local /home/pats/dependencies/ | true
		mkdir -p /usr/local/
		chown pats /usr/local/ -R
	else # ubu 20
        cp /home/pats/dependencies/grub_20 /etc/default/grub
        rm  /home/pats/dependencies/grub_20
		apt-mark hold grub-efi-amd64
		apt remove -y sendmail
        update-grub
		# sed -i 's/^\(.*OnlyOnACPower\s*=\s*\)"true";/\1"false";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\s*\/\/\s*Unattended-Upgrade::OnlyOnACPower\s*"true";/Unattended-Upgrade::OnlyOnACPower "true";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\(.*Skip-Updates-On-Metered-Connections\s*=\s*\)"true";/\1"false";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		# sed -i 's/^\s*\/\/\s*Unattended-Upgrade::Skip-Updates-On-Metered-Connections\s*"true";/Unattended-Upgrade::Skip-Updates-On-Metered-Connections "true";/g' /etc/apt/apt.conf.d/50unattended-upgrades
		unattended-upgrade -d | true
        apt remove -y librealsense2-dkms
        apt upgrade -y
        #apt-mark unhold grub-efi-amd64

        mkdir -p /home/pats/dependencies
		mv /usr/local /home/pats/dependencies/ | true
		mkdir -p /usr/local/
		chown pats /usr/local/ -R
    fi
	apt install -y avrdude gdisk
	echo Please reboot me.
	touch $RELEASE_UPGRADE_P1_FLAG
fi
