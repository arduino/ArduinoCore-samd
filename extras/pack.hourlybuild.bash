#!/bin/bash -ex

BUILD_NUMBER=$1
CURR_TIME=`date "+%Y-%m-%d %H:%M"`
CURR_TIME_SED=`date "+%Y\\-%m\\-%d %H:%M"`
VERSION=9.9.9-Hourly

PWD=`pwd`
FOLDERNAME=`basename $PWD`
THIS_SCRIPT_NAME=`basename $0`
FILENAME=package_samd-hourly-b${BUILD_NUMBER}.tar.bz2

rm -f $FILENAME

# Change name in platform.txt
sed -i "s/name=.*/name=SAMD Hourly Build ${BUILD_NUMBER} (${CURR_TIME})/" platform.txt

cd ..
tar --transform "s|$FOLDERNAME|samd-hourly_b${BUILD_NUMBER}|g"  --exclude=extras/** --exclude=.git* --exclude=.idea -cjf $FILENAME $FOLDERNAME
cd -

mv ../$FILENAME .

CHKSUM=`sha256sum $FILENAME | awk '{ print $1 }'`
SIZE=`wc -c $FILENAME | awk '{ print $1 }'`

cat extras/package_index.json.Hourly.template |
sed "s/%%BUILD_NUMBER%%/${BUILD_NUMBER}/" |
sed "s/%%CURR_TIME%%/${CURR_TIME_SED}/" |
sed "s/%%VERSION%%/${VERSION}/" |
sed "s/%%FILENAME%%/${FILENAME}/" |
sed "s/%%CHECKSUM%%/${CHKSUM}/" |
sed "s/%%SIZE%%/${SIZE}/" > package_samd-hourly-build_index.json

