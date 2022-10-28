#! /usr/bin/env bash
python3 /home/pats/pats/release/scripts/hostname.py || true
python3 /home/pats/pats/release/scripts/timezone.py || true
/bin/su - pats -c "/usr/bin/screen -dm -S daemon ~/pats/release/scripts/screens.sh"
