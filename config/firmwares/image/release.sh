#!/usr/bin/env bash

set -e

UBUNTU_18_SYSTEM=pats242
UBUNTU_20_SYSTEM=pats8

cd ~/code/trapeye/
git fetch
if [ "$(git rev-parse HEAD)" != "$(git rev-parse @{u})" ] || [ -n "$(git status --porcelain)" ]; then
  echo "Trap-Eye repo has unfinished business"
  exit 1
fi
cd ~/code/pats/
if [ "$(git rev-parse HEAD)" != "$(git rev-parse @{u})" ] || [ -n "$(git status --porcelain)" ]; then
  echo "PATS repo has unfinished business"
  exit 1
fi

cp -r ~/code/pats/base/install ~/code/release-18/
cp -r ~/code/pats/base/scripts ~/code/release-18/
cp -r ~/code/pats/base/xml ~/code/release-18/
cp ~/code/trapeye/firmware/trapeye.bin ~/code/release-20/firmware/trapeye/

cp -r ~/code/pats/base/install ~/code/release-20/
cp -r ~/code/pats/base/scripts ~/code/release-20/
cp -r ~/code/pats/base/xml ~/code/release-20/
cp ~/code/trapeye/firmware/trapeye.bin ~/code/release-18/firmware/trapeye/


set -x
cd ~/code/release-18/build
rsync ${UBUNTU_18_SYSTEM}:code/pats/base/build/executor ./ -az
rsync ${UBUNTU_18_SYSTEM}:code/trapeye/basestation/build/trapeye ./ -az
cd ..
#rsync ${UBUNTU_18_SYSTEM}:dependencies/binaries.tar.gz ./ -az

cd ~/code/release-20/build
rsync ${UBUNTU_20_SYSTEM}:code/pats/base/build/executor ./ -az
rsync ${UBUNTU_20_SYSTEM}:code/trapeye/basestation/build/trapeye ./ -az
cd ..
#rsync ${UBUNTU_20_SYSTEM}:dependencies/binaries.tar.gz ./ -az

set +ex
echo *----------*
echo Current commit:
git log -1 --pretty=%B
echo *----------*
echo Current tag:
git describe --tags
echo *----------*
echo Specify new release tag:
read tagname


cd ~/code/pats
git tag $tagname
git push --tags
cd ~/code/trapeye
git tag $tagname
git push --tags

cd ~/code/release-18/
git ci -am $tagname
git tag $tagname
git push
git push --tags

cd ~/code/release-20/
git ci -am $tagname
git tag $tagname
git push
git push --tags

