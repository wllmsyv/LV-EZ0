/*
 * Test Bench:
 * 		Arduino Mega 2650
 * 		Maxbotix LV-EZ0 MB1000 Sonar Range finder 6"-255"
 *
 *	Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 *
 *
 *	This program reads data from a sonar style range sensor
 *	and displays it through the serial monitor.
 *
 * New data arrives approximately every 49.4ms
 * pulse width = 147us/in @ 5 Vcc
 *
 *  For Reading PW:
 *   __________________________
 *   LV-EZ0 	|  Board
 *   --------------------------
 * 	 Pin 2 (PW)	|  Inerrupt Pin
 * 	 Pin 6 		|  Vcc
 * 	 Pin 7 		|  GND
 *   --------------------------
 *
 *
 *
 *  For Reading Serial:
 *   __________________________
 *   LV-EZ0 	|  Board
 *   --------------------------
 *   Pin 1 (PW)	|  GND
 *   Pin 5 (TX)	|  Pin(19) RX1 		Note: The serial line needs to be inverted.
 *              |          				  Baud rate  set too 9600
 * 	 Pin 6 		|  Vcc
 * 	 Pin 7 		|  GND
 *   --------------------------
 */



#include "Arduino.h"

/*=============================================
 *
 *		Local Variables
 *=============================================
 */

uint8_t range_pin = 2;

long time_1 = 0;
unsigned long distance_pwm_inches = 0;
String distance_serial_inches = "";
uint32_t byte_read_count = 0;

bool serial_data_ready = false;
bool pwm_data_ready = false;

/*=============================================
 *
 *		Setup
 *=============================================
 */
void setup()
{
	Serial.begin(115200);
	Serial1.begin(9600);
	pinMode(range_pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(range_pin), range_interrupt, CHANGE);
}

/*=============================================
 *
 *		Main
 *=============================================
 */
void loop()
{
	if(pwm_data_ready) {
		pwm_data_ready = false;
		Serial.print("PWM Data:\t\t");Serial.println(distance_pwm_inches);
	}
	if(serial_data_ready) {
		serial_data_ready = false;
		Serial.print("Serial Data:\t");Serial.println(distance_serial_inches);
	}
}


/*=============================================
 *
 *		Interrupt Functions
 *=============================================
 */

// measure the time distance by measuring
// the distance between pulse edges.
void range_interrupt(void) {

	if(digitalRead(range_pin)) {

		time_1 = micros();
		pwm_data_ready = false;

	} else {

		distance_pwm_inches = ((micros()-time_1)/147.0);
		pwm_data_ready = true;

	}
};


// Use serial interrupt to read in distance
// in ascii over serial communication.
void serialEvent1(){

	char c = 0x0;

	while(Serial1.available() > 0) {
		c = (char)Serial1.read();

		// The first character to come in is an ascii 'R' (0x52)
		if(c == (char)0x52){
			byte_read_count++;
			distance_serial_inches = "";
			serial_data_ready = false;

		// Reset after 4 characters have been received
		} else if(byte_read_count > 3) {
			byte_read_count = 0;
			serial_data_ready = true;

		// Distance comes in as 3 ascii characters
		} else if(byte_read_count < 4 && byte_read_count > 0) {
			distance_serial_inches = distance_serial_inches +c;
			byte_read_count++;
		}
	}
};
