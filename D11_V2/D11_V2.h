// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef D11_V2_H_
#define D11_V2_H_
#include "Arduino.h"
//add your includes for the project D11_V2 here

#include "HardwareSerial.h"
#include "math.h"

#include "UltrasonicProximitySensor.h" 	// ULTRASONIC SENSOR
#include "DifferencialMotors.h" 		// DIFFERENTIAL MOTORS
#include "Buzzer.h" 					// BUZZER
#include "LED.h" 						// LED
#include "Radar.h"						// RADAR (SERVO + IR)

#include "Event.h"						// ARDUINO EVENTS
#include "TimerOne.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

enum Status {
	scanning_objects,
	reaching_biggest_object,
	random_movement
};

void reach_biggest_object(float obj_distance, int obj_direction);
void random_move_for(unsigned long milliseconds);

//Do not add code below this line
#endif /* D11_V2_H_ */
