#!/usr/bin/env bash

if [ ! -d builddir ]
then
    mkdir builddir
    echo "build directory created"
fi

/usr/local/arduino-1.6.6/arduino-builder \
 -compile \
 -build-path "$(pwd)/builddir" \
 -logger=human \
 -hardware "/usr/local/arduino-1.6.6/hardware" \
 -tools "/usr/local/arduino-1.6.6/tools-builder" \
 -tools "/usr/local/arduino-1.6.6/hardware/tools/avr" \
 -built-in-libraries "/usr/local/arduino-1.6.6/libraries" \
 -fqbn=arduino:avr:uno \
 -ide-version=10606 \
 -warnings=none \
 -prefs=build.warn_data_percentage=75 \
 *ino

exit $?
