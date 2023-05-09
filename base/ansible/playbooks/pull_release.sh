#!/usr/bin/env bash

set -e
[ ! -f /home/pats/dependencies/release_upgraded_20230405.done ] || {
unset SSH_AUTH_SOCK
unset SSH_CLIENT
unset SSH_CONNECTION
unset SSH_TTY
export HOME=/home/pats/
cd ~/pats/release
git pr
kill $(pgrep -f daemon.py)
kill $(pgrep -f baseboardlink.py)
}
