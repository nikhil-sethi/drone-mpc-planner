#! /usr/bin/env bash

set -e

./autostart.sh &
ssh -i /home/slamdunk/kevin/mu-g/id_rsa -fNR 16666:localhost:22 houjebek@dinstech.nl  &

