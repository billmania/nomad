#!/usr/bin/env bash

/usr/local/arduino-1.6.6/arduino \
 --upload \
 --verbose-upload \
 *ino

exit $?
