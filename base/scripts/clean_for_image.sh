#!/usr/bin/env bash
set -ex

sudo hostnamectl set-hostname pats0
sudo rm /etc/wireguard/wg0.conf -rf

rm ~/dependencies/hostname_set -rf
rm ~/dependencies/timezone_set -rf
rm ~/dependencies/wireguard_set -rf
rm ~/pats/data/* -rf
rm ~/pats/images/* -rf
rm ~/pats/flags/* -rf
rm ~/pats/jsons/* -rf
rm ~/pats/logs/* -rf
rm ~/pats/renders/* -rf
rm ~/pats/status/* -rf
rm ~/pats/xml/* -rf
cp ~/pats/release/install/.bash_history ~/.bash_history

touch ~/pats/flags/disable  ~/pats/flags/disable_baseboard  ~/pats/flags/disable_charging ~/pats/flags/disable_trapeye

rm .cache/ -rf
rm .ccache/ -rf
# rm .vscode-server/ -rf # only run when necessary, and initialize vscode afterwards again

sudo unattended-upgrade