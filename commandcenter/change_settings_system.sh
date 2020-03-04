#!/usr/bin/env bash
set -ex

rsync -z tmp.xml $1:code/pats/xml/pats_deploy.xml

./restart_system.sh $1