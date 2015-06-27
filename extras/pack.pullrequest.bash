#!/bin/bash -ex

PR_NUMBER=$1
BUILD_NUMBER=$2
VERSION=`grep version= platform.txt | sed 's/version=//g'`

PWD=`pwd`
FOLDERNAME=`basename $PWD`
THIS_SCRIPT_NAME=`basename $0`
FILENAME=package_samd-b${BUILD_NUMBER}.tar.bz2

rm -f $FILENAME

# Change name in platform.txt
sed -i "s/name=.*/name=SAMD Pull request #${PR_NUMBER} (Build ${BUILD_NUMBER})/" platform.txt

cd ..
tar --transform "s|$FOLDERNAME|samd-PR${PR_NUMBER}_b${BUILD_NUMBER}|g"  --exclude=extras/** --exclude=.git* --exclude=.idea -cjf $FILENAME $FOLDERNAME
cd -

mv ../$FILENAME .

CHKSUM=`sha256sum $FILENAME | awk '{ print $1 }'`
SIZE=`wc -c $FILENAME | awk '{ print $1 }'`

cat extras/package_index.json.PR.template |
sed s/%%PR_NUMBER%%/${PR_NUMBER}/ |
sed s/%%BUILD_NUMBER%%/${BUILD_NUMBER}/ |
sed s/%%VERSION%%/${VERSION}-build-${BUILD_NUMBER}/ |
sed s/%%FILENAME%%/${FILENAME}/ |
sed s/%%CHECKSUM%%/${CHKSUM}/ |
sed s/%%SIZE%%/${SIZE}/ > package_samd-b${BUILD_NUMBER}_index.json

