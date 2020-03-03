#!/usr/bin/env bash
set -ex

rsync -z ../xml/flightplans/simple_demo.xml $1:pats_demo.xml