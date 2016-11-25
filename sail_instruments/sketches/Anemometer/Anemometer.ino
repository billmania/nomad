/*
 * Anemometer
 *
 * Read the wind speed and direction from a Davis
 * anemometer. Write a WIMWV NMEA sentence to the
 * console and to the CAN bus.
 *
 * http://www.davisnet.com/product_documents/weather/spec_sheets/6410_SS.pdf
 *
 * Bill Mania <bill@manialabs.us>
 *
 * Yellow - 5V
 * Red    - Gnd
 * Green  - Direction (pin A0)
 * Black  - Speed (pin 2)
 */

#include <stdio.h>
/*
#include <mcp2515.h>
 */

#define MICROS_PER_SECOND 1000000UL
#define MAX_NMEA_LENGTH 100

int anemometerInterrupt = 0;
int directionPin = 0;
int directionVoltage, windSource, windSpeedKnots;
volatile unsigned long currentTime, previousTime;
volatile double elapsedTime;
volatile unsigned long pulses, latestPulses;
int sentenceLength;
char buffer[MAX_NMEA_LENGTH], checksum;
/*
tCAN messageOut, messageIn;
 */

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
  Serial.println(F("Anemometer"));
  Serial.println(F("Version 10"));

  /*
   * Initialize the bus at 500 Kbps
  if (mcp2515_init(1)) {
    Serial.println(F("CAN bus init OK"));
  } else {
    Serial.println(F("mcp2515_init() failed"));
  }
   */

  previousTime = micros();
  pulses = 0UL;
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(
    anemometerInterrupt,
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

    /*
     * Write data to CAN bus
    if (mcp2515_check_free_buffer()) {
        messageOut.id = 0x1;
        messageOut.header.rtr = (int8_t) 0;
        messageOut.header.length = (uint8_t) 1;
        messageOut.data[0] = (uint8_t) 0;

        if (! mcp2515_send_message(&messageOut)) {
            Serial.println(F("Failed to send CAN message"));
        }
    } else {
        Serial.println(F("No space for outbound message"));
    }

    if (mcp2515_check_message()) {
        Serial.println(F("CAN message ready"));
        mcp2515_get_message(&messageIn);
        Serial.println(messageIn.id);
    }
     */

    delay(1000);
}

