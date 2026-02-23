#!/bin/bash

BUILD_TYPE=${1:-Release}

if [ "$BUILD_TYPE" != "Debug" ] && [ "$BUILD_TYPE" != "Release" ]; then
    echo "Error: Build type must be 'Debug' or 'Release'."
    exit 1
fi

echo "=== Configuring CMake for $BUILD_TYPE ==="
cmake -S . --preset $BUILD_TYPE

echo "=== Building project ($BUILD_TYPE) ==="
cmake --build build/$BUILD_TYPE --preset $BUILD_TYPE
