#!/bin/bash
#
#
# https://gist.github.com/Farious/e841ef85a8f4280e4f248ba8037ea2c0
#
# Script that will use the provided Android App Bundle (.aab) and change its version code and version name to the provided values, avoiding re-building the whole .aab.
# Run this script with: sh rollback.sh your_project.aab android_signing_key.key key_alias key_pwd version_code version_name
# 
# Necessary setup:
# 
# jarsigner - This binary should exist in the path
#
# Configuration.proto and Resources.proto can be found in aapt2's github
#  https://github.com/aosp-mirror/platform_frameworks_base/tree/master/tools/aapt2/Configuration.proto
#  https://github.com/aosp-mirror/platform_frameworks_base/tree/master/tools/aapt2/Resources.proto
# 
# PROTO_BIN - The location of the protoc - Google's protobuf compiler
#  Swap <proto_bin_folder> for your setup's protoc folder
# 
# ZIP_ALIGN - The location of Android's zipalign tool found on:
#  <Android_sdk_path>/build-tools/<build_version>/zipalign
#
# BUNDLE_TOOL - The location of Google's oficial bundletool. This dependency is just to generate the relevant .apk to verify that everything worked properly.
#               Download from:
#  https://github.com/google/bundletool/releases
# 
if [ $# -ne 6 ] ; then
    echo "Not enough parameters supplied"
fi

# Begin - Setup necessary
PROTO_BIN=protoc
ZIP_ALIGN=zipalign
BUNDLE_TOOL=./bundletool-all-1.14.0.jar
# End - Setup

# Inputs
ORIGINAL_AAB=$1
ANDROID_SIGNING_KEY=$2
KEY_ALIAS=$3
SIGN_KEY_PWD=$4
VERSION_CODE=$5
VERSION_NAME=$6

SOURCE_FOLDER="${ORIGINAL_AAB%.*}"
DEST_FOLDER=`basename $ORIGINAL_AAB .aab`
OUT_FILE=$DEST_FOLDER.aab
TEMP_FILE=temp_$OUT_FILE

echo "Unpacking AAB - $ORIGINAL_AAB"
unzip $ORIGINAL_AAB -d $DEST_FOLDER

echo "Decoding AndroidManifest.xml"
$PROTO_BIN --decode=aapt.pb.XmlNode --proto_path=tools Configuration.proto Resources.proto < $DEST_FOLDER/base/manifest/AndroidManifest.xml > AndroidManifest_temp.xml

if [ $? != 0 ] ; then
    echo "Decoding AndroidManifest.xml failed."
fi

echo "Change Version Code"
REGEX=`echo 's/(.*[vV]ersionCode"\\n.*")[0-9]+("\\n)((?:.*\\n)*?)(.*int_decimal_value: )[0-9]+/$1 . '$VERSION_CODE' . $2 . $3 . $4 . '$VERSION_CODE'/eg'`
perl -0777 -i.backup -pe "$REGEX" AndroidManifest_temp.xml

echo "Change Version Name"
REGEX=`echo 's/(.*[vV]+ersionName"\\n.*value: )"[0-9\.]+"/$1"'$VERSION_NAME'"/g'`
perl -0777 -i.backup -pe "$REGEX" AndroidManifest_temp.xml

cp ~/code/pats/config/firmwares/app/splashscreen_image.png $DEST_FOLDER/base/res/drawable/splashscreen_image.png

echo "Encoding edited AndroidManifest.xml"
rm -rf $DEST_FOLDER/base/manifest/AndroidManifest.xml
$PROTO_BIN --encode=aapt.pb.XmlNode --proto_path=tools Configuration.proto Resources.proto < AndroidManifest_temp.xml > $DEST_FOLDER/base/manifest/AndroidManifest.xml
rm -rf AndroidManifest_temp.xml
rm -rf AndroidManifest_temp.xml.backup

if [ $? != 0 ] ; then
    echo "Encoding edited AndroidManifest.xml failed."
fi

echo "Generating"
pushd $DEST_FOLDER
rm -rf ./base/root/META-INF
rm -rf ./META-INF
zip -r -D ../$TEMP_FILE *
popd

if [ $? != 0 ] ; then
    echo "zipping failed"
fi

echo "Processing"
set -x
jarsigner -verbose -keystore $ANDROID_SIGNING_KEY -storepass $SIGN_KEY_PWD $TEMP_FILE $KEY_ALIAS

if [ $? != 0 ] ; then
    echo "jarsigner failed"
fi

$ZIP_ALIGN -f -v -p 4 $TEMP_FILE $OUT_FILE

if [ $? != 0 ] ; then
    echo "zipalign failed"
fi

echo "Validation"
java -Djava.io.tmpdir=$PWD -jar $BUNDLE_TOOL build-apks --bundle $OUT_FILE --mode universal --output rollback.apks --ks $ANDROID_SIGNING_KEY --ks-pass "pass:$SIGN_KEY_PWD" --ks-key-alias "$KEY_ALIAS" --key-pass "pass:$SIGN_KEY_PWD"

if [ $? != 0 ] ; then
    echo "bundletool failed to generate apk"
fi

rm -rf $TEMP_FILE $DEST_FOLDER rollback.apks

