#!/usr/bin/env bash
set -ex

cd
rsync dash:pats_ssh_files_v4.tar.gz ./ -az
rsync dash:pats_git_config /home/pats/code/pats/.git/config -az
if [ -f pats_ssh_files_v4.tar.gz ] ; then
    rm ~/.ssh -rf
    mkdir -p .ssh
    mv pats_ssh_files_v4.tar.gz .ssh/
    cd .ssh
    tar -xf pats_ssh_files_v4.tar.gz
    cd
    cd code/pats
    git fetch
else
    echo WARNING SSH FILES NOT FOUND
fi