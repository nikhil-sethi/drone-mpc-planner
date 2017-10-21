#! /usr/bin/env bash

set -e

./autostart.sh &
ssh -TR 16666:localhost:22 houjebek@dinstech.nl  &
