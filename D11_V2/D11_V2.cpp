// Do not remove the include below
#include "D11_V2.h"

// === PIN DEI COMPONENTI ===
// Sensore ad ultrasuoni
#define ECHO_PIN 4
#define TRIGGER_PIN 3
// Radar (Sensore ad infrarossi + Servomotore)
#define IR_PIN 0
#define SERVO_PIN 11
// Buzzer
#define MELODY_PIN 9
// LED
#define GREEN_LED_PIN 2
#define RED_LED_PIN 10
// Motori
#define LEFT_MOTOR_ENABLE_PIN 5
#define LEFT_MOTOR_LOGIC_PIN_1 12
#define LEFT_MOTOR_LOGIC_PIN_2 13
#define RIGHT_MOTOR_ENABLE_PIN 6
#define RIGHT_MOTOR_LOGIC_PIN_1 7
#define RIGHT_MOTOR_LOGIC_PIN_2 8

// === PARAMETRI DI FUNZIONAMENTO ===
#define ACCEPTED_ERROR 5 	// Distanza tollerata di errore nelle misurazioni delle distanze
#define SAFE_DISTANCE 15 	// Distanza di sicurezza per i movimenti del robot
#define TARGET_ACHIEVED 20	// Distanza minima per il raggiungimento dell'obiettivo

extern HardwareSerial Serial;

struct Object
{
	Object() : direction(NULL), dimension(NULL) {}
	Object(float dir, float dim) : direction(dir), dimension(dim) {}
	float direction;
	float dimension;
};

// === SENSORI COMPONENTI ===
DifferencialMotors diff_motors;
LED led_green;
LED led_red;
Buzzer buzzer;
Radar radar;
UltrasonicProximitySensor sonar;

// === GESTORE DEGLI EVENTI & EVENTO "OSTACOLO RILEVATO" ===
EventManager evtManager;
Event ev_obstacle("obstacle_detected");

// === VARIABILI DI UTILITA' ===
Object biggest_object;
volatile boolean biggest_object_found = false;
volatile boolean biggest_object_reached = false;
volatile boolean abort_mission = false;
volatile Status current_status;
volatile int obstacle_detected_time = 0;

// === METODO PER LA GESTIONE DELL'EVENTO "OGGETTO TROVATO" ===
struct EvObjectFoundObserver : public EventTask
{
	using EventTask::execute;

	void execute(Event evt)
	{
		// Emissione del segnale sonoro e luminoso
		buzzer.bip_sound();
		led_green.blink();

		// Gestione dei parametri descrittivi dell'oggetto
		String obj_info = (String)evt.extra;

		int delimeter = obj_info.indexOf("-");
		String dir = obj_info.substring(0, delimeter);
		String dim = obj_info.substring(delimeter+1);

		char dir_array[dir.length() + 1];
		dir.toCharArray(dir_array, sizeof(dir_array));
		float direction = atof(dir_array);

		char dim_array[dim.length() + 1];
		dim.toCharArray(dim_array, sizeof(dim_array));
		float dimension = atof(dim_array);

		// Analisi delle dimensioni dell'oggetto trovato
		if (biggest_object_found) {
		  if (biggest_object.dimension > dimension) {
			  return;
		  }
		}

		biggest_object_found = true;
		biggest_object = Object(direction, dimension);

	}

} EvObjectFoundObserver;

// === METODO PER LA GESTIONE DELL'EVENTO "OSTACOLO RILEVATO" ===
struct EvObstacleDetectedObserver : public EventTask
{
	using EventTask::execute;

	void execute(Event evt)
	{
		diff_motors.stop();

		// Emissione del segnale luminoso
		led_red.blink();

		/* Se il robot rimane bloccato da un ostacolo per più di
		 * 5 secondi, si decide di abbandonare la mission corrente
		 * ed effettuare dei movimenti casuali per 30 secondi
		 */
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
			/* Se lo stato attuale del sistema è quello di avvicinamento
			 * all'oggetto più grande, è necessario controllare se è
			 * un semplice ostacolo o è stato raggiunto l'obiettivo */
			if (radar.checkObjectReached(TARGET_ACHIEVED)){
				biggest_object_reached = true;
			}
		} else if (current_status == random_movement) {
			/* Se lo stato attuale del sistema è quello di effettuare
			 * movimenti casuali, si deve verificare da quale lato è
			 * disponibile una maggiore area di manovra */
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

// === METODO INVOCATO DAL TIMER INTERRUPT
void checkObstacles(){
	interrupts();
	if (sonar.distance_cm() < SAFE_DISTANCE && !biggest_object_reached) {
		/* Se il sonar frontale rileva una distanza minore di quella di sicurezza
		 * viene innescato l'evento "OSTACOLO RILEVATO" mettendo in pausa il timer
		 * fino a che l'evento non viene gestito completamente */
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

	// === INIZIALIZZAZIONE DEI COMPONENTI ===

	diff_motors.setLeftMotorPins(LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_LOGIC_PIN_1, LEFT_MOTOR_LOGIC_PIN_2);
	diff_motors.setRightMotorPins(RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_LOGIC_PIN_1, RIGHT_MOTOR_LOGIC_PIN_2);
	diff_motors.setBackwheelType(chair_caster_wheel);

	led_green.setPin(GREEN_LED_PIN);
	led_red.setPin(RED_LED_PIN);

	buzzer.setPin(MELODY_PIN);

	radar.initWithPin(IR_PIN, SERVO_PIN);

	sonar.initWithPins(TRIGGER_PIN, ECHO_PIN);

	// Sottoscrizione
	evtManager.subscribe(Subscriber("object_found", &EvObjectFoundObserver));
	evtManager.subscribe(Subscriber("obstacle_detected", &EvObstacleDetectedObserver));

	radar.addObserver(evtManager);

	// Inizializzazione Timer interrupt 1 Arduino
	Timer1.initialize(150000); // 150000 = 0.15 seconds
	Timer1.attachInterrupt(checkObstacles); // checkObstacles to run every 0.15 seconds
}

void loop()
{
	// Inizializzazione variabili di controllo
	biggest_object_found = false;
	biggest_object_reached = false;
	abort_mission = false;

	// Configurazione iniziale
	led_green.setOFF();
	led_red.setOFF();

	// Cambio di stato intero => Ricerca dell'oggetto più grande
	current_status = scanning_objects;
	// cerca l'oggetto più grande davanti

	radar.scan();

	if (biggest_object_found) {
		// Se è stato trovato l'oggetto più grande

		// Cambio di stato intero => Avvicinamento all'oggetto più grande
		current_status = reaching_biggest_object;

		// Segnale luminoso per indicare lo stato attuale
		led_green.setON();

		// Controllo della distanza dell'oggetto
		float big_obj_distance = radar.distanceAtPosition_cm((int)biggest_object.direction);

		// Avvio dei movimenti verso l'oggetto trovato
		reach_biggest_object(big_obj_distance, biggest_object.direction);
	}

	if (biggest_object_found && biggest_object_reached) {
		// E' stato trovato E raggiunto l'oggetto più grande

		// MISSIONE COMPLETATA: Emissione segnale sonoro di successo
		buzzer.success_sound();

	} else if (!abort_mission) {
		// Non è stato trovato alcun oggetto e la missione non è stata abortita

		// MISSIONE NON COMPLETATA: Emissione segnale sonoro di fallimento
		buzzer.fail_sound();

	}

	// Cambio di stato intero => Movimenti casuali
	current_status = random_movement;

	// Impostazione del segnale luminoso per indicare lo stato attuale
	led_green.setOFF();
	led_red.setON();

	// Movimenti casuali per 30.000 millisecondi (= 30 secondi)
	random_move_for(30000);

	// Attesa di un secondo prima di ripartire da capo
	delay(1000);
}

void reach_biggest_object(float obj_distance, int obj_direction)
{
	// Rotazione verso l'oggetto più grande trovato
	int turn_angle = 90 - obj_direction;
	diff_motors.turn(turn_angle);

	// Movimento verso l'oggetto con controllo con retroazione
	while(!biggest_object_reached && biggest_object_found){
		// Fino a che l'oggetto non è stato raggiunto E non è stato perduto
		while (radar.distanceAtPosition_cm(90) > obj_distance && abs(turn_angle) > 10) {
			/* Fino a che la distanza frontale rilevata dal radar è maggiore della
			 * distanza nota dell'oggetto E l'ultima rotazione è inferiore ai 10 gradi*/

			// Controlla la direzione dell'oggetto ed effettua la relativa rotazione
			turn_angle = radar.checkObjectDirection(obj_distance, turn_angle/2, 15);
			diff_motors.turn(turn_angle);
		}
		// Quando l'oggetto è di fronte al robot effettua un movimento in avanti per 300 millisecondi
		diff_motors.goForward();
		delay(300);
		diff_motors.stop();
	}
}

void random_move_for(unsigned long milliseconds)
{
	biggest_object_reached = false;
	int turn_choice = 5;
	unsigned long start = millis();
	float distance;
	while (millis() < start + milliseconds) {
		// Per [milliseconds] millisecondi ...
		// Scegli casualmente un movimento
		int choice = random(-turn_choice,turn_choice+1); // 0,1,2 -> avanti, -5 -> gira a sinistra, 5 -> gira a destra
		if (abs(choice) == turn_choice) {
			// Movimento scelto: Rotazione di 90° a destra o a sinistra
			choice = choice / turn_choice;
			int angle = 90 * choice;

			if (radar.distanceAtPosition_cm(90 - angle) > SAFE_DISTANCE){
				// Se c'è spazio a sufficenza per effettuare una rotazione, ruota
				diff_motors.turn(angle);
			}

		} else {
			// Movimento in avanti
			radar.set_position(90);
			diff_motors.goForward();
			diff_motors.stop();
		}


	}
	// Motori fermi
	diff_motors.stop();
}
