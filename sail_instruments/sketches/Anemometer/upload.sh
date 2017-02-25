#!/usr/bin/env bash

ARDUINO_DIR="/usr/local/arduino-1.6.13"
PROJECTS_DIR="/home/bill/projects/nomad/sail_instruments/sketches"
PROJECT="Anemometer"

cd ${PROJECTS_DIR}/${PROJECT}

if [ -d builddir ]
then
    rm -rf builddir
fi
mkdir builddir

${ARDUINO_DIR}/arduino \
 --upload \
 --verbose-upload \
 --preserve-temp-files \
 --port /dev/ttyACM0 \
 --board arduino:avr:mega:cpu=atmega2560 \
 --pref build.path=builddir \
 ${PROJECTS_DIR}/${PROJECT}/${PROJECT}.ino

exit $?
