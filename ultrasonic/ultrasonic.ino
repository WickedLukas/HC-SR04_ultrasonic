/*
* ultrasonic.ino
*
* Created: 1.12.2018
* Author: Lukas
*
* This sketch is based on:
* http://homediyelectronics.com/projects/arduino/arduinoprogramminghcsr04withinterrupts/
* 
* TimerOne library:
* https://github.com/PaulStoffregen/TimerOne
* 
*/

#include "Arduino.h"
#include "TimerOne.h"

#define HCSR04_TRIGGER_PIN 8	// pin connected to HC-SR04 trigger
#define HCSR04_ECHO_PIN 7 		// pin connected to HC-SR04 echo

#define TRIGGER_TIME 20							// time in microseconds trigger is set to high
#define TRIGGER_TIME_INTERVAL	2000	// maximum trigger time interval in TRIGGER_TIME

volatile uint16_t trigger_time_count = 0;	// counter for TRIGGER_TIME_INTERVAL
volatile uint8_t state = 1;	// state variable
	
volatile int32_t echo_start;			// echo start in microseconds
volatile int32_t echo_end;				// echo end in microseconds
volatile int32_t echo_duration;		// echo duration in microseconds

volatile boolean timer_interrupt = false;
volatile boolean echo_interrupt = false;

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

void timer_isr() {
	if (++trigger_time_count >= TRIGGER_TIME_INTERVAL) {
		trigger_time_count = 0;	// reset trigger_time_count
	 	state = 1;
	}
	
	switch(state) {
		case 0:		// idle state
			break;
  	case 1:		// start trigger pulse
  		digitalWrite(HCSR04_TRIGGER_PIN, HIGH);	// set trigger pin to high
  		state = 2;
  		break;
  	case 2:		// finish trigger pulse
  		digitalWrite(HCSR04_TRIGGER_PIN, LOW);	// set trigger pin to low
			state = 0;
			break;
	}

	timer_interrupt = true;
}

void echo_isr() {
	switch (digitalRead(HCSR04_ECHO_PIN)) {
		case HIGH:	// echo pin change was a rising edge (start of echo pulse)
			echo_start = micros();
      break;
    case LOW:		// echo pin change was a falling edge (end of echo pulse)
    	echo_end = micros();
    	echo_duration = echo_end - echo_start;	// calculate echo pulse duration
    	//trigger_time_count = 0;
    	//state = 1;
    	break;
  }
	echo_interrupt = true;
}

void setup() {
  #ifdef DEBUG
	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo eNUMeration, others continue immediately
	#endif

	pinMode(HCSR04_TRIGGER_PIN, OUTPUT);		// configure trigger pin as output
	digitalWrite(HCSR04_TRIGGER_PIN, LOW);	// set trigger pin to low
	pinMode(HCSR04_ECHO_PIN, INPUT);	// configure echo pin as input
	attachInterrupt(digitalPinToInterrupt(HCSR04_ECHO_PIN), echo_isr, CHANGE);	// attach interrupt service routine echo_isr to echo pin

	Timer1.initialize(TRIGGER_TIME);		// initialise timer 1
  Timer1.attachInterrupt(timer_isr);	// attach interrupt service routine timer_isr to timer 1
}

void loop() {
	static float distance;	// distance to obstacle in centimeter

  if (echo_interrupt) {
		distance = echo_duration * 0.01716;
		if (distance < 2000){
			DEBUG_PRINTLN(distance);
		}
		echo_interrupt = false;
	}
}
