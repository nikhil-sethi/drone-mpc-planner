#!/usr/bin/env bash
set -ex

sudo apt install ifupdown
sudo apt-get remove `cat packages_toberemoved.txt` --purge
sudo apt autoremove
sudo snap remove sublime-text code gnome-system-monitor gnome-logs gnome-characters gnome-calculator gnome-3-34-1804 gnome-3-28-1804
sudo snap remove gtk-common-themes
sudo snap remove core20 core18


