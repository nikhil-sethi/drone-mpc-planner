#!/bin/bash

TRIGGER_DELAY=60

while true; do
    touch ~/pats/flags/virtual_insect
    sleep $TRIGGER_DELAY
done
