#! /usr/bin/env bash
python3 /home/pats/pats/release/scripts/hostname.py || true
python3 /home/pats/pats/release/scripts/timezone.py || true
/bin/su - pats -c "/usr/bin/screen -dm -S daemon ~/pats/release/scripts/screens.sh"
[ -f /home/pats/pats/flags/disable_trapeye ] || {
/home/pats/pats/release/scripts/trapeye_wifi_boot_setup.sh
}
