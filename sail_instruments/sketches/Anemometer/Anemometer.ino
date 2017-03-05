/*
 * Anemometer
 *
 * Read the wind speed and direction from a Davis
 * anemometer. Write a WIMWV NMEA sentence to the
 * console and to the CAN bus.
 *
 * https://bitbucket.org/fmalpartida/new-liquidcrystal
 *
 * http://www.davisnet.com/product_documents/weather/spec_sheets/6410_SS.pdf
 *
 * https://www.sainsmart.com/arduino/arduino-shields/lcd-shields/sainsmart-iic-i2c-twi-serial-2004-20x4-lcd-module-shield-for-arduino-uno-mega-r3.html
 *
 * http://www.gammon.com.au/interrupts
 *
 * Bill Mania <bill@manialabs.us>
 *
 * Yellow - 5V
 * Red    - Gnd
 * Green  - Direction (pin A0)
 * Black  - Speed (pin 2)
 */

#define DISPLAY
#define VERSION F("Anemometer v1.7")

#include <stdio.h>

#ifdef DISPLAY
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR      0x27 // I2C address of PCF8574A
#define BACKLIGHT_PIN 3
#define En_pin        2
#define Rw_pin        1
#define Rs_pin        0
#define D4_pin        4
#define D5_pin        5
#define D6_pin        6
#define D7_pin        7

LiquidCrystal_I2C twilcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin, BACKLIGHT_PIN, POSITIVE);

#endif

long currentMillis, previousMillis;
#define MICROS_PER_SECOND 1000000UL
#define MAX_NMEA_LENGTH 100

int anemometerPin = 2; // digital
int directionPin = 0;  // analog
int directionVoltage, windSource, windSpeedKnots;
volatile unsigned long currentTime, previousTime;
volatile double elapsedTime;
volatile unsigned long pulses;
unsigned long latestPulses;
int sentenceLength;
char buffer[MAX_NMEA_LENGTH], checksum;

void irInterrupt(void) {
    pulses++;
};

char calcChecksum(char *sentence) {
    byte checksum = 0;

    while (*sentence) {
        checksum ^= *sentence++;
    }

    return char(checksum);
}

void
setup()
{
  //Setup usb serial connection to computer
  Serial.begin(9600);
  Serial.println(VERSION);

#ifdef DISPLAY
  twilcd.begin(20, 4);
  twilcd.clear();
  twilcd.print(VERSION);
  delay(1000);
  twilcd.clear();
  twilcd.setCursor(0, 0);
  twilcd.print("Speed ");
  twilcd.setCursor(0, 1);
  twilcd.print("Direction ");
#endif

  previousTime = micros();
  pulses = 0UL;
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(anemometerPin),
    irInterrupt,
    FALLING
    );
  interrupts();
}

void
loop() {

    latestPulses = pulses; pulses = 0UL;
    currentTime = micros();
    elapsedTime = currentTime - previousTime;
    windSpeedKnots = latestPulses * (2.25 / (elapsedTime / MICROS_PER_SECOND)) * 0.868976;
    previousTime = currentTime;

    directionVoltage = analogRead(directionPin);
    windSource = int(0.351745 * directionVoltage - 0.054602);

    sentenceLength = sprintf(
        buffer,
        "WIMWV,%d,R,%d,K,A",
        windSource,
        windSpeedKnots
        );

    if (sentenceLength < MAX_NMEA_LENGTH) {
        Serial.print("$");
        Serial.print(buffer);
        Serial.println(calcChecksum(buffer));
    };


#ifdef DISPLAY
    twilcd.setCursor(6, 0);
    twilcd.print(windSpeedKnots);
    twilcd.print("      ");
    twilcd.setCursor(10, 1);
    twilcd.print(windSource);
    twilcd.print("      ");
#endif

    delay(1000);
}

