#!/usr/bin/env bash

ARDUINO_DIR="/usr/local/arduino-1.6.9"
PROJECTS_DIR="/home/bill/projects"
PROJECT="nomad/sail_instruments/sketches/Anemometer"

# -libraries "/home/bill/sketchbook/libraries" \
# -tools "${ARDUINO_DIR}/hardware/tools/avr" \

cd ${PROJECTS_DIR}/${PROJECT}

if [ -d builddir ]
then
    rm -rf builddir
fi

mkdir builddir
${ARDUINO_DIR}/arduino-builder \
 -compile=true \
 -logger=human \
 -hardware "/home/bill/.arduino15/packages" \
 -hardware "${ARDUINO_DIR}/hardware" \
 -tools "/home/bill/.arduino15/packages" \
 -tools "${ARDUINO_DIR}/tools-builder" \
 -built-in-libraries "${ARDUINO_DIR}/libraries" \
 -fqbn=arduino:avr:mega:cpu=atmega2560 \
 -core-api-version=10609 \
 -build-path "${PROJECTS_DIR}/${PROJECT}/builddir" \
 -warnings=more \
 -prefs=build.warn_data_percentage=75 \
 -verbose=true \
 "${PROJECTS_DIR}/${PROJECT}/Anemometer.ino"

exit $?
