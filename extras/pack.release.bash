#!/bin/bash -ex

#  pack.*.bash - Bash script to help packaging samd core releases.
#  Copyright (c) 2015 Arduino LLC.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

# Version check removed because version string passed from jenkins was incorrect
VERSION_FROM_TAG=$1
CORE_NAME=$2
echo $VERSION_FROM_TAG
echo $CORE_NAME
VERSION=`grep version= platform.txt | sed 's/version=//g'`
echo $VERSION

if [ $VERSION != $VERSION_FROM_TAG ]; then
    exit 0
fi

PWD=`pwd`
FOLDERNAME=`basename $PWD`
THIS_SCRIPT_NAME=`basename $0`
FILENAME=core-$CORE_NAME-$VERSION.tar.bz2
echo $FILENAME

rm -f *.tar.bz2
rm -f *.json

cd ..
tar  --exclude=extras/** --exclude=.git* --exclude=.idea -cjhf $FILENAME $FOLDERNAME
cd -

mv ../$FILENAME .

CHKSUM=`sha256sum $FILENAME | awk '{ print $1 }'`
SIZE=`wc -c $FILENAME | awk '{ print $1 }'`

cat extras/package_index.json.NewTag.template |
# sed "s/%%BUILD_NUMBER%%/${BUILD_NUMBER}/" |
# sed "s/%%CURR_TIME%%/${CURR_TIME_SED}/" |
sed "s/%%VERSION%%/${VERSION}/" |
sed "s/%%FILENAME%%/${FILENAME}/" |
sed "s/%%CHECKSUM%%/${CHKSUM}/" |
sed "s/%%SIZE%%/${SIZE}/" > package_${CORE_NAME}_${VERSION}_index.json