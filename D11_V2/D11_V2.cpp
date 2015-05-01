// Do not remove the include below
#include "D11_V2.h"

#define ECHO_PIN 4
#define TRIGGER_PIN 3
#define SERVO_PIN 11
#define IR_PIN 0
#define MELODY_PIN 9
#define GREEN_LED_PIN 2
#define RED_LED_PIN 10

#define ACCEPTED_ERROR 5
#define TARGET_ACHIEVED 20
#define SAFE_DISTANCE 15
#define MAX_OBJECTS 10

#define DEBUG 1

extern HardwareSerial Serial;

struct Object
{
	Object() : direction(NULL), dimension(NULL) {}
	Object(float dir, float dim) : direction(dir), dimension(dim) {}
	float direction;
	float dimension;
};

// Sensors components
DifferencialMotors diff_motors;
LED led_green;
LED led_red;
Buzzer buzzer;
Radar radar;
UltrasonicProximitySensor sonar;

// Events Handler
EventManager evtManager;

Event ev_obstacle("obstacle_detected");

// Useful stuff
Object biggest_object;
volatile boolean biggest_object_found = false;
volatile boolean biggest_object_reached = false;
volatile boolean abort_mission = false;
volatile Status current_status;
volatile int obstacle_detected_time = 0;

struct EvObjectFoundObserver : public EventTask
{
	using EventTask::execute;

	void execute(Event evt)
	{
		buzzer.bip_sound();
		led_green.blink();

		String obj_info = (String)evt.extra;

		int delimeter = obj_info.indexOf("-");
		String dir = obj_info.substring(0, delimeter);
		String dim = obj_info.substring(delimeter+1);

		char dir_array[dir.length() + 1]; //determine size of the array
		dir.toCharArray(dir_array, sizeof(dir_array)); //put readStringinto an array
		float direction = atof(dir_array);

		char dim_array[dim.length() + 1]; //determine size of the array
		dim.toCharArray(dim_array, sizeof(dim_array)); //put readStringinto an array
		float dimension = atof(dim_array);

		if (biggest_object_found) {
		  if (biggest_object.dimension > dimension) {
			  return;
		  }
		}

		biggest_object_found = true;
		biggest_object = Object(direction, dimension);

	}

} EvObjectFoundObserver;

struct EvObstacleDetectedObserver : public EventTask
{
	using EventTask::execute;

	void execute(Event evt)
	{
		diff_motors.stop();

		led_red.blink();

		if (obstacle_detected_time == 0){
			obstacle_detected_time = millis();
		} else if (millis() - obstacle_detected_time > 5000){
			if (current_status == scanning_objects){
				radar.abort_scan();
			}
			biggest_object_found = false;

			abort_mission = true;
			radar.set_position(90);
			led_green.setOFF();
			buzzer.fail_sound();

			current_status = random_movement;
		}

		if (current_status == reaching_biggest_object) {
			if (radar.checkObjectReached(TARGET_ACHIEVED)){
				biggest_object_reached = true;
			}
		} else if (current_status == random_movement) {
			// bisogna controllare da che parte è possibile girarsi
			float left_distance = radar.distanceAtPosition_cm(180);
			float right_distance = radar.distanceAtPosition_cm(0);
			if (left_distance > SAFE_DISTANCE || right_distance > SAFE_DISTANCE) {
				if (left_distance > right_distance) {
					diff_motors.turn(-90);
				} else {
					diff_motors.turn(90);
				}
			} else {
				diff_motors.goBackward();
				diff_motors.stop();
			}

		}

	}

} EvObstacleDetectedObserver;

void checkObstacles(){
	interrupts();
	if (sonar.distance_cm() < SAFE_DISTANCE && !biggest_object_reached) {
		Timer1.stop();
		evtManager.trigger(ev_obstacle);
		Timer1.restart();
	} else {
		obstacle_detected_time = 0;
	}
}

void setup()
{
	Serial.begin(9600);

	diff_motors.setLeftMotorPins(5, 12, 13);
	diff_motors.setRightMotorPins(6, 7, 8);
	diff_motors.setBackwheelType(chair_caster_wheel);

	led_green.setPin(GREEN_LED_PIN);
	led_red.setPin(RED_LED_PIN);

	buzzer.setPin(MELODY_PIN);

	radar.initWithPin(IR_PIN, SERVO_PIN);

	sonar.initWithPins(TRIGGER_PIN, ECHO_PIN);

	evtManager.subscribe(Subscriber("object_found", &EvObjectFoundObserver));
	radar.addObserver(evtManager);

	evtManager.subscribe(Subscriber("obstacle_detected", &EvObstacleDetectedObserver));

	Timer1.initialize(150000); // 150000 = 0.15 seconds
	Timer1.attachInterrupt(checkObstacles); // checkObstacles to run every 0.15 seconds
}

void loop()
{
	biggest_object_found = false;
	biggest_object_reached = false;
	abort_mission = false;

	// led spenti
	led_green.setOFF();
	led_red.setOFF();

	current_status = scanning_objects;
	// cerca l'oggetto più grande davanti

	radar.scan();

	if (biggest_object_found) {

		current_status = reaching_biggest_object;

		// sono stati trovati degli oggetti
		led_green.setON();

		// puntiamo l'oggetto
		float big_obj_distance = radar.distanceAtPosition_cm((int)biggest_object.direction);
		// cerchiamo di raggiungere l'oggetto trovato
		reach_biggest_object(big_obj_distance, biggest_object.direction);

	}

	if (biggest_object_found && biggest_object_reached) {

		// festeggia!
		buzzer.success_sound();

		led_green.setOFF();

	} else if (!abort_mission) {

		// non è stato trovato o raggiunto alcun oggetto

		buzzer.fail_sound();

	}

	led_red.setON();
	// muoviti a caso per un po' di passi (evitando gli ostacoli) per poi fare una nuova scansione

	current_status = random_movement;

	random_move_for(30000);

	delay(1000);

}

void reach_biggest_object(float obj_distance, int obj_direction)
{
	// ruota in direzione dell'oggetto più grande
	int turn_angle = 90 - obj_direction;
	diff_motors.turn(turn_angle);

	while(!biggest_object_reached && biggest_object_found){
		while (radar.distanceAtPosition_cm(90) > obj_distance && abs(turn_angle) > 10) {
			turn_angle = radar.checkObjectDirection(obj_distance, turn_angle/2, 15);
			diff_motors.turn(turn_angle);
		}
		diff_motors.goForward();
		delay(300);
		diff_motors.stop();
	}
}

void random_move_for(unsigned long milliseconds)
{
	biggest_object_reached = false;
	int turn_choice = 20;
	unsigned long start = millis();
	float distance;
	while (millis() < start + milliseconds) {
		int choice = random(-turn_choice,turn_choice+1); // 0,1,2 -> avanti, -5 -> gira a sinistra, 5 -> gira a destra
		if (abs(choice) == turn_choice) {

			choice = choice / turn_choice;
			int angle = 90 * choice;

			distance = radar.distanceAtPosition_cm(90 - angle);

			if (distance > SAFE_DISTANCE){
				diff_motors.turn(angle);
			}

		} else {
			radar.set_position(90);
			diff_motors.goForward();
			diff_motors.stop();
		}


	}
	diff_motors.stop();
}
