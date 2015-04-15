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
// LINE SENSOR

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

void go_until_obstacle();

int ultrasonic_distance();

int search_obj(float obj_distance, int last_turn_angle, int wide);
void set_servo_position(int degree);
void go_towards_object(float obj_distance, int obj_direction);
float scan();

void random_move_for(unsigned long milliseconds);
//Do not add code below this line
#endif /* D11_V2_H_ */
