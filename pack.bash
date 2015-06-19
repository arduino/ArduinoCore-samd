#!/bin/bash -ex

VERSION=`grep version= platform.txt | sed 's/version=//g'`

PWD=`pwd`
FOLDERNAME=`basename $PWD`
THIS_SCRIPT_NAME=`basename $0`

rm -f samd-$VERSION.tar.bz2

cd ..
tar --transform "s|$FOLDERNAME|$VERSION|g"  --exclude=.git --exclude=.idea --exclude=$THIS_SCRIPT_NAME -cjf samd-$VERSION.tar.bz2 $FOLDERNAME
cd -

mv ../samd-$VERSION.tar.bz2 .

