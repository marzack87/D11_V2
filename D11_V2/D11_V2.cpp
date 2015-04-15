// Do not remove the include below
#include "D11_V2.h"

#define ECHO_PIN 4
#define TRIGGER_PIN 3
#define SERVO_PIN 5
#define IR_PIN 0
#define MELODY_PIN 9
#define GREEN_LED_PIN 2
#define RED_LED_PIN 10

#define SAFE_DISTANCE 25
#define ACCEPTED_ERROR 5
#define TARGET_ACHIEVED 10

extern HardwareSerial Serial;

DifferencialMotors diff_motors;
LED led_green;
LED led_red;
Buzzer buzzer;
Radar radar;
UltrasonicProximitySensor front_sonar;
EventManager evtManager;

struct ObjFoundListener : public EventTask
{
  using EventTask::execute;

  void execute(Event evt)
  {
    buzzer.bip_sound();
  }

} ObjFoundListener;

void setup()
{
  Serial.begin(9600);

  diff_motors.setLeftMotorPins(11, 12, 13);
  diff_motors.setRightMotorPins(6, 7, 8);
  diff_motors.setBackwheelType(chair_caster_wheel);

  led_green.setPin(GREEN_LED_PIN);
  led_red.setPin(RED_LED_PIN);

  buzzer.setPin(MELODY_PIN);

  radar.initWithPin(IR_PIN, SERVO_PIN);

  front_sonar.initWithPins(TRIGGER_PIN, ECHO_PIN);

  evtManager.subscribe(Subscriber("object_found", &ObjFoundListener));

  radar.addListener(evtManager);
}

void loop()
{
	radar.scan();

	/*
	// led spenti
	led_green.setOFF();
	led_red.setOFF();

	// cerca l'oggetto più grande davanti
	float big_obj_direction = radar.scan();
	delay(1000);
	if (big_obj_direction == -1) {

		// non è stato trovato alcun oggetto
		led_red.setON();

		buzzer.fail_sound();

	} else {
		// è stato trovato l'oggetto
		led_green.setON();

		// puntiamo l'oggetto
		float big_obj_distance = radar.distanceAtPosition_cm(big_obj_direction);

		// andiamo verso l'oggetto
		go_towards_object(big_obj_distance, big_obj_direction);

		// festeggia!
		buzzer.success_sound();

		led_green.setOFF();

	}

	led_red.setON();
	// muoviti a caso per un po' di passi (evitando gli ostacoli) per poi fare una nuova scansione

	random_move_for(30000);

	delay(1000);*/
}

void go_towards_object(float obj_distance, int obj_direction)
{
	// ruota in direzione dell'oggetto più grande trovato
	int turn_angle = 90 - obj_direction;

	diff_motors.turn(turn_angle);

	float ahead_distance;
	int stop;

	while (front_sonar.distance_cm() > TARGET_ACHIEVED && obj_distance > TARGET_ACHIEVED) {

		// controlliamo la distanza di fronte al robot
		ahead_distance = radar.distanceAtPosition_cm(90);

		Serial.print("Distanza di fronte al robot: ");
		Serial.println(ahead_distance);

		float temp_distance;
		stop = 0;

		/*
		 * 	fino a che la distanza di fronte al robot è maggiore
		 * 	(considerando anche 5 cm di errore) a quella dell'oggetto
		 * 	che abbiamo registrato girando il servo
		 */
		while (ahead_distance > obj_distance + ACCEPTED_ERROR &&
				obj_distance > TARGET_ACHIEVED) {

			// cerchiamo l'oggetto visto che non è davanti a noi
			/*
			 * turn_angle / 2 è per dare un'idicazione sulla direzione in cui
			 * è più probabile che sia presente l'oggetto che cerchiamo, facendo
			 * riferimento all'ultimo angolo di rotazione
			 */
			turn_angle = search_obj(obj_distance, turn_angle/2, 15);

			// aggiorniamo la distanza dell'oggetto
			temp_distance = radar.distanceAtPosition_cm(90 - turn_angle);
			if (temp_distance < obj_distance + ACCEPTED_ERROR) obj_distance = temp_distance;

			// ruotiamo verso l'oggetto
			if (turn_angle < 10){
				stop = 1;
				turn_angle *= 1.5;
			}
			diff_motors.turn(turn_angle);
			Serial.print(".Ho girato di: ");
			Serial.println(turn_angle);
			delay(500);

			// controlliamo la distanza davanti a noi
			ahead_distance = radar.distanceAtPosition_cm(90);

			Serial.print(".Distanza di fronte al robot: ");
			Serial.println(ahead_distance);

			if (stop == 1) break;
		}

		if (obj_distance > TARGET_ACHIEVED){
			// andiamo in avanti
			turn_angle = 0;
			Serial.println("Avanti tutta!");
			diff_motors.goForward();
			delay(200);
			diff_motors.stop();
		}
	}
	diff_motors.stop();
}

int search_obj(float obj_distance, int estimated_direction, int wide)
{
	/*
	 * estimated_direction indica la direzione verso cui è più
	 * probabile che ci sia l'oggetto, è la metà dell'angolo
	 * dell'ultima giro del robot, serve per evitare di fare una
	 * scansione eccessiva ma guardare subito nella
	 * direzione in cui dovrebbe esserci l'oggetto
	 *
	 */

	int start = 0;
	int end = 0;

	start = (90 - estimated_direction) - wide;
	end = (90 - estimated_direction) + wide;

	if (start < 0) start = 0;
	if (end > 180) end = 180;

	float servo_distance;
	int new_angle = -1;
	for(int pos = start; pos < end; pos += 2) {
		servo_distance = radar.distanceAtPosition_cm(pos);
		if (servo_distance < obj_distance + ACCEPTED_ERROR){
			new_angle = 90 - pos;
			break;
		}
	}
	if (new_angle == -1) search_obj(obj_distance, estimated_direction, wide + 10);
	return new_angle;
}

void random_move_for(unsigned long milliseconds)
{
	unsigned long start = millis();
	float distance;
	while (millis() < start + milliseconds) {
		int choice = random(-3,4); // 0 -> avanti, -1 -> gira a sinistra, 1 -> gira a destra
		Serial.println(choice);
		if (abs(choice) == 3) {
			choice = choice / 3;
			int angle = 90 * choice;

			distance = radar.distanceAtPosition_cm(90 - angle);
			delay(100);
			if (distance > SAFE_DISTANCE){
				diff_motors.turn(angle);
			}
		} else {

			radar.set_radar_position(90);
			if (front_sonar.distance_cm() > SAFE_DISTANCE) {
				diff_motors.goForward();
				delay(250);
				diff_motors.stop();
				delay(300);
			}
		}
	}
}
