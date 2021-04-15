#!/bin/bash
make package/symlinks
make defconfig
make package/symlinks
echo "build -j$(nproc) (ignore errors)"
(time IGNORE_ERRORS=1 make -j$(nproc) V=s FORCE_UNSAFE_CONFIGURE=1 ) |& tee IGNORE_ERRORS.log
echo "build -j1"
(time make -j1 V=s FORCE_UNSAFE_CONFIGURE=1 ) |& tee after_IGNORE_ERRORS.log


