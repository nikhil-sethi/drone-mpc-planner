#!/bin/sh

cut -b 10- $HOME/code/pats/config/firmwares/charging/baseboard.ino.eightanaloginputs.hex | rev | cut -c 4- | rev | tr -d '\n'  | xxd -r -p | strings | grep vers_stmp
