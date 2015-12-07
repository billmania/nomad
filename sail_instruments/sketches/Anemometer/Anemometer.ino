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
#define MAXIMUM_UPDATE_PERIOD 1000000UL

int anemometerInterrupt = 0;
int directionPin = 0;
int position = 0;
int directionVoltage, windSource;
volatile unsigned long currentTime, previousTime, elapsedTime;
int rpm, rpmArray[] = {0, 0, 0, 0, 0};

void
irInterrupt() {
    currentTime = micros();
    if (currentTime > previousTime) {
        elapsedTime = currentTime - previousTime;
    } else {
        elapsedTime = 1L;
    };
    previousTime = currentTime;
};

void
setup()
{
  //Setup usb serial connection to computer
  Serial.begin(9600);
  Serial.println("Anemometer");
  Serial.println("Version 3");

  previousTime = micros();
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
    if ((micros() - previousTime) > MAXIMUM_UPDATE_PERIOD) {
        // The shaft is rotating too slowly, or not at all
        Serial.println(0);
    } else {
        if (elapsedTime > 1) {
		    position = (position + 1) % 5;
		    rpmArray[position] = int(MICROS_PER_MINUTE / elapsedTime);
		
		    rpm = (rpmArray[0] + rpmArray[1] + rpmArray[2] +rpmArray[3] + rpmArray[4]) / 5;
            Serial.println(rpm);
        } else {
            Serial.println(" Neg ");
        };
    };

    directionVoltage = analogRead(directionPin);
    if (directionVoltage > 919) {
        windSource = 0;
    } else if (directionVoltage > 902) {
        windSource = 45;
    } else if (directionVoltage > 878) {
        windSource = 90;
    } else if (directionVoltage > 849) {
        windSource = 135;
    } else if (directionVoltage > 805) {
        windSource = 180;
    } else if (directionVoltage > 715) {
        windSource = 225;
    } else if (directionVoltage > 550) {
        windSource = 270;
    } else {
        windSource = 315;
    }

    Serial.print("Source ");
    Serial.println(windSource);

    delay(1000);
}

