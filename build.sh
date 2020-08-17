#!/bin/sh

set -x

SOURCE_DIR=$(cd "$(dirname "$0")" || exit;pwd)
BUILD_TYPE=${BUILD_TYPE:-release}
BUILD_DIR=${BUILD_DIR:-cmake-build-${BUILD_TYPE}}
INSTALL_DIR=${INSTALL_DIR:-install}

mkdir -p $BUILD_DIR \
  && cd $BUILD_DIR \
  && cmake \
           -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
           -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
           -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
           $SOURCE_DIR \
  && make $*
