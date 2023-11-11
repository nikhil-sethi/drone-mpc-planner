#!/bin/bash

TRIGGER_DELAY=3

while true; do
    touch ~/pats/flags/static_target
    sleep $TRIGGER_DELAY
done
