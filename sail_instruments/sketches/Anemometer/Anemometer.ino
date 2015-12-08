/*
 * Anemometer
 *
 * Read the wind speed and direction from a Davis
 * anemometer.
 * http://www.davisnet.com/product_documents/weather/spec_sheets/6410_SS.pdf
 *
 * Bill Mania <bill@manialabs.us>
 */

#define MICROS_PER_MINUTE 60000000UL
#define MICROS_PER_SECOND 1000000UL
#define MAXIMUM_UPDATE_PERIOD 1000000UL

int anemometerInterrupt = 0;
int directionPin = 0;
int position = 0;
int directionVoltage, windSource, windSpeedKnots;
volatile unsigned long currentTime, previousTime;
volatile double elapsedTime;
volatile unsigned long pulses, latestPulses;
int rpm, rpmArray[] = {0, 0, 0, 0, 0};

void
irInterrupt(void) {
    pulses++;
};

void
setup()
{
  //Setup usb serial connection to computer
  Serial.begin(9600);
  Serial.println("Anemometer");
  Serial.println("Version 5");

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
    /*
     * This loop requires about 500 micros. Reading the clock
     * requires about 4 micros.
     */
    latestPulses = pulses; pulses = 0UL;
    currentTime = micros();
    elapsedTime = currentTime - previousTime;
    windSpeedKnots = latestPulses * (2.25 / (elapsedTime / MICROS_PER_SECOND)) * 0.868976;
    previousTime = currentTime;

    directionVoltage = analogRead(directionPin);
    windSource = int(0.351745 * directionVoltage - 0.054602);

    Serial.print("Source ");
    Serial.print(windSource);
    Serial.print(" Knots ");
    Serial.println(windSpeedKnots);

    delay(1000);
}

