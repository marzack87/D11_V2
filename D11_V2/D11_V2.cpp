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
}

void loop()
{
	// led spenti
	led_green.setOFF();
	led_red.setOFF();

	// cerca l'oggetto più grande davanti
	float big_obj_direction = scan();
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

	delay(1000);
}

void go_towards_object(float obj_distance, int obj_direction)
{
	// ruota in direzione dell'oggetto più grande trovato
	int turn_angle = 90 - obj_direction;

	diff_motors.turn(turn_angle);

	float ahead_distance;
	int stop;

	while (ultrasonic_distance() > TARGET_ACHIEVED && obj_distance > TARGET_ACHIEVED) {

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
			if (ultrasonic_distance() > SAFE_DISTANCE) {
				diff_motors.goForward();
				delay(250);
				diff_motors.stop();
				delay(300);
			}
		}
	}
}

int ultrasonic_distance()
{
  float average;
  float total = 0;

  int read_per_pos = 5;
  for(int i = 0; i < read_per_pos; i++) {

    delay(50);
    int distance = front_sonar.distance_cm();
    total += distance;
  }

  average = total / read_per_pos;

  return average;
}

float scan()
{
  float prevDist = 0;

  float begin_dist = 0;
  float end_dist = 0;

  int firstEdge = 0;
  float firstEdge_dist = 0;
  int firstEdge_angle = 0;

  int secondEdge = 0;
  float secondEdge_dist = 0;
  int secondEdge_angle = 0;

  float average;

  double obj_dimension = 0;
  int obj_number = 0;

  float deg = 0;
  float rad = 0;
  float angle_direction = 0;

  int big_obj_number = 0;
  double big_obj_dimension = 0;
  float big_obj_direction = -1;

  int pos = 0;

  for(pos = 0; pos < 180; pos += 2) {

    average = radar.distanceAtPosition_cm(pos);

    Serial.print("Angolo: ");
    Serial.print(pos);
    Serial.print(" Distance: ");
    Serial.println(average);

    if (pos != 0) {
      /*
          Se c'è uno scalino (distanza precendente - distanza attuale)
          > 10 cm  ---> abbiamo appena incontrato il primo "bordo" di un oggetto
          > -10 cm ---> abbiamo appena incontrato il secondo "bordo" di un oggetto [l'oggetto è terminato, bisogna calcolarne le dimensioni]

     */
      if (prevDist - average > 10){

        firstEdge_dist = average;
        firstEdge_angle = pos;

        firstEdge = 1;
        Serial.print("################");
        Serial.print("Trovato primo Spigolo");
        Serial.println("################");

      }

      if (average - prevDist > 10){
        if (firstEdge == 1) {

          secondEdge_dist = prevDist;
          secondEdge_angle = pos-2;

          secondEdge = 1;
          Serial.println("################");
          Serial.println("Trovato secondo Spigolo");
          Serial.println("################");

        }
      }

      if (firstEdge == 1 && secondEdge == 1){

        deg = secondEdge_angle - firstEdge_angle;
        rad =  (deg * 71) / 4068;

        angle_direction = (firstEdge_angle + secondEdge_angle) / 2;

        obj_dimension = sqrt( pow(firstEdge_dist, 2) + pow(secondEdge_dist, 2) - 2 * firstEdge_dist * secondEdge_dist * cos( rad ) );

        if (obj_dimension > 3.0){

          obj_number++;

          Serial.println("**************");
          Serial.print("Oggetto numbero: ");
          Serial.println(obj_number);
          Serial.print("Distanza primo bordo: ");
          Serial.println(firstEdge_dist);
          Serial.print("Distanza primo bordo: ");
          Serial.println(firstEdge_dist);
          Serial.print("Distanza secondo bordo: ");
          Serial.println(secondEdge_dist);
          Serial.print("Ampiezza angolo compreso: ");
          Serial.print(deg);
          Serial.print(" gradi (");
          Serial.print(rad);
          Serial.println(" radianti)");
          Serial.print("Angolo di direzione: ");
          Serial.println(angle_direction);
          Serial.print("Dimensione Oggetto: ");
          Serial.println(obj_dimension);
          Serial.println("**************");

          led_green.setON();
          delay(100);
          led_green.setOFF();

          buzzer.bip_sound();

          led_green.setOFF();
          delay(100);
          led_green.setON();

          if (big_obj_dimension < obj_dimension) {
            big_obj_dimension = obj_dimension;
            big_obj_number = obj_number;
            big_obj_direction = angle_direction;
          }
        }

        firstEdge = 0;
        secondEdge = 0;

      }

    prevDist = average;

    if (pos == 180) {

      end_dist = average;

    }

  } else {

    begin_dist = average;
    prevDist = average;
  }

 }
  delay(1000);

  Serial.println("--------------------------------");
  Serial.println("OBIETTIVO INDIVIDUATO");
  Serial.print("[ Oggetto : ");
  Serial.print(big_obj_number);
  Serial.print(", Diametro : ");
  Serial.print(big_obj_dimension);
  Serial.print(", Angolo : ");
  Serial.println(big_obj_direction);
  Serial.println(" ]");
  Serial.println("A MORTE IL CICCIONE!!");
  Serial.println("--------------------------------");

  return big_obj_direction;

}
