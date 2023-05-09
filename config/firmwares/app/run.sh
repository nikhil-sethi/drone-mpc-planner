#!/bin/bash
# usage: copy the aab to this folder and run
# the publish key and pass can be found in thunkable project settings under "Android keystore"

# set -e
# read -p 'apk path: ' apkpath
# apktool d $apkpath -o apk


read -p 'aab path: ' aabpath
#read -sp 'Key pass: ' passvar
passvar=$(<.pass)
read -p 'Version: ' version


./rollback_aab.sh $aabpath android.keystore androidkey $passvar $version v$version

