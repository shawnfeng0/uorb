#!/usr/bin/env sh

SOURCE_DIR=$(
  cd "$(dirname $0)" || exit
  pwd
)

# Get the platform name from the first parameter
if [ $# -ge 1 ]; then
  PLATFORM_NAME=$1
else
  echo "USAGE: $0 [platform_name]"
fi

# Get platform toolchain info
PLATFORM_NAME=${PLATFORM_NAME:-host.gcc}
PLATFORM_TOOLCHAIN_FILE=$SOURCE_DIR/toolchains/$PLATFORM_NAME.toolchain.cmake

# Check if the platform toolchain exists
if [ ! -f $PLATFORM_TOOLCHAIN_FILE ]; then
  echo "Can't find platform toolchain file: $PLATFORM_TOOLCHAIN_FILE"
  exit 1
fi

# Build library
set -x
rm -rf $SOURCE_DIR/cmake-build-$PLATFORM_NAME
mkdir -p "$SOURCE_DIR/cmake-build-$PLATFORM_NAME" &&
  cd "$SOURCE_DIR/cmake-build-$PLATFORM_NAME" &&
  cmake \
    -DCMAKE_TOOLCHAIN_FILE="$PLATFORM_TOOLCHAIN_FILE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    .. &&
  make -j4 &&
  make install
