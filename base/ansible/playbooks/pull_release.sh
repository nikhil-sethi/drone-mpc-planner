#!/usr/bin/env bash

set -e
unset SSH_AUTH_SOCK
unset SSH_CLIENT
unset SSH_CONNECTION
unset SSH_TTY
export HOME=/home/pats/
cd ~/pats/release
git pr
