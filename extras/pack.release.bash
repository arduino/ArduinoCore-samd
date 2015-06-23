#!/bin/bash -ex

VERSION=`grep version= platform.txt | sed 's/version=//g'`

PWD=`pwd`
FOLDERNAME=`basename $PWD`
THIS_SCRIPT_NAME=`basename $0`

rm -f samd-$VERSION.tar.bz2

cd ..
tar --transform "s|$FOLDERNAME|$FOLDERNAME-$VERSION|g"  --exclude=extras/** --exclude=.git* --exclude=.idea -cjf samd-$VERSION.tar.bz2 $FOLDERNAME
cd -

mv ../samd-$VERSION.tar.bz2 .

