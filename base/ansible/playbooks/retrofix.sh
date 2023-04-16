#!/usr/bin/env bash

set -e
[ ! -f /home/pats/dependencies/retro_fix_20230416.done ] || {
    exit 0
}
[ ! -f /home/pats/dependencies/release_upgraded_20230405.done ] || {
    unset SSH_AUTH_SOCK
    unset SSH_CLIENT
    unset SSH_CONNECTION
    unset SSH_TTY
    export HOME=/home/pats/
    cd ~/pats/release
    git pr
    cd scripts
    ./retro_fix_size.py
    echo Please reboot me
}