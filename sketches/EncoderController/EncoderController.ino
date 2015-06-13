/*
 * Controller for two quadrature encoders
 *
 * Bill Mania <bill@manialabs.us>
 *
 * Left encoder is assumed to be mounted "backwards"
 *
 */
#include <Encoder.h>
#define VERSION "V3"
#define PUBLISH_HZ 5 

const unsigned long updateInterval = 1000 / PUBLISH_HZ;

Encoder encoderLeft(2, 3);
Encoder encoderRight(18, 19);

void setup() {
  Serial.begin(9600);
}

long positionLeft = -999;
long positionRight = -999;
long newLeft, newRight;
boolean somethingChanged = false;
long maxPosition = 65000;

void loop() {
  
  newLeft = -(encoderLeft.read());
  
  if (newLeft > maxPosition) {
    newLeft = newLeft - maxPosition;
    encoderLeft.write(newLeft);
  } else if (newLeft < -(maxPosition)) {
    newLeft = newLeft + maxPosition;
    encoderLeft.write(newLeft);
  }

  newRight = encoderRight.read();
  if (newRight > maxPosition) {
    newRight = newRight - maxPosition;
    encoderLeft.write(newRight);
  } else if (newRight < -(maxPosition)) {
    newRight = newRight + maxPosition;
    encoderLeft.write(newRight);
  }

  if (newLeft != positionLeft) {
    somethingChanged = true;
    positionLeft = newLeft;
  }

  if (newRight != positionRight) {
    somethingChanged = true;
    positionRight = newRight;
  }
  
  if (somethingChanged) {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(newLeft);
    Serial.print(",");
    Serial.print(newRight);
    Serial.print(",");
    Serial.print(VERSION);
    Serial.println();

    somethingChanged = false;
  }

  if (Serial.available()) {
    Serial.read();
    encoderLeft.write(0);
    encoderRight.write(0);
  }

  delay(updateInterval);
}
