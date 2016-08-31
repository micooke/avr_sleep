#include <Arduino.h>

/*
Power save stuff based off:
Low Power Testing
Spark Fun Electronics 2011
Nathan Seidle
https://www.sparkfun.com/tutorials/309

Low power blinky test for 8MHz boards
Note: This will work with other clock frequencies - but the delay routine will be wrong
*/

#include <avr_sleep.h>

avr_sleep attiny85;

#define LED_PIN 3

void setup()
{
	// setup the attiny85 to sleep
	attiny85.setup();
	digitalWrite(2, HIGH);
	attiny85.sleep_int0();

	// setup the LED
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	//attiny85.sleep_wdt(6);
	attiny85.sleep_ms(2000);
	digitalWrite(LED_PIN, LOW);
}

void loop()
{
	attiny85.sleep_wdt(5); 
	digitalWrite(LED_PIN, HIGH);
	attiny85.sleep_wdt(5); 
	digitalWrite(LED_PIN, LOW);
}