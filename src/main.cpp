/*
 * *****************************************************************************
 * SPRING PRELOADER
 * *****************************************************************************
 * 
 * Michael Wettstein
 * Juli 2024, Zürich
 * 
 * *****************************************************************************
 * Program to control an assembly aid device that preloads a spring by turning
 * a threaded connection using a stepper motor.
 * 
 * The spring must always be compressed to the same length and the machine
 * must always stop at the desired angle.
 * 
 * PLC runtime should be small, preferably <100us, so that angle measurements
 * are precise enough.
 * 
 * TURNS | NEW POS | ACTION 
 *  0.0       -      MOTOR STARTS TO TURN
 *  4.0  ->  0.0     SENSOR DETECTS PLC SETS NEW POSITION
 *  8.5 -----------> MOTOR STOP POSITION IF SENSOR DID NOT DETECT
 * (8.8)     4.8     DESIRED PRECISE MOTOR STOP SPRING LENGTH 37-37.5mm
 * 11.0 -----------> MECHANICAL CRASH !!!
 * ---> EXACT VALUES ARE TO BE FOUND IN UPC COMMANDER MOTOR CONTROLLER AND
 * ---> rig-spring-preloader.xls SPREADSHEET TAB "turn calculator"
 * *****************************************************************************
 * COMPONENTS
 * *****************************************************************************
 * 
 * PLC ----> Controllino Maxi --------> monitors sensors for safety and position
 * DRIVER -> LAM DS3078 Motor Driver -> accelerates and decelerates stepper motor
 * MOTOR --> NEMA34 Stepper ----------> turns screw joint to compress spring to 
 *                                      desired length and angle
 * 
 * DOOR ---> Sensors to detect if safety door is closed
 * LENGTH -> Sensor to detect if thread is at a certain length
 * ANGLE --> Sensor to detect a fixed angle at every motor turn
 * 
 * *****************************************************************************
 * PROGRAM STRUCTURE
 * *****************************************************************************
 * ----------------
 * NORMAL OPERATION
 * ----------------
 * •) USER CLOSES DOOR
 * 
 * •) PLC checks if DOOR is closed and LENGTH sensor does not detect yet
 * 
 * •) PLC enables DRIVER, DRIVER accelerates MOTOR
 * 
 * •) PLC detects LENGTH -> PLC waits for next ANGLE
 *
 * •) PLC detects ANGLE -> DRIVER calculates and sets new target position
 *
 * •) DRIVER stops MOTOR at target position
 * 
 * •) USER opens DOOR -> PLC resets
 * 
 * --------------  
 * SPECIAL EVENTS
 * --------------
 * 
 * •) DOOR opens while motor is running -> PLC disables MOTOR, MOTOR stopps immediately!
 * 
 * •) LENGTH sensor detects already when user closes door
 * -> Spring already compressed, MOTOR must not start to turn
 * 
 * *****************************************************************************
 * RUNTIME ON CONTROLLINO MAXI
 * 2024-07-30 MIN12 AVG14 MAX20 [us]
 * *****************************************************************************
 *  * 
 * *****************************************************************************
 * ----
 * TODO
 * ----
 * • -
 * *****************************************************************************
 * 
 * 
 */

#include <Arduino.h>
#include <Controllino.h>
#include <Debounce.h> //   https://github.com/chischte/debounce-library
#include <Insomnia.h>

Insomnia print_delay;

// GLOBAL VARIABLES ------------------------------------------------------------

// I/O-PINS:
// INPUTS:

// OUTPUT PINS:
const byte MOTOR_ENABLE = CONTROLLINO_D0; // ----------- LAM MOTOR CONTROLLER PIN CN3 - PIN1 (DI0)
const byte MOTOR_SET_NEW_POSITION = CONTROLLINO_D1; // - LAM MOTOR CONTROLLER PIN CN3 - PIN3 (DI1)

// INPUT PINS
const byte MOTOR_STOPPED = CONTROLLINO_A5; // -------- LAM MOTOR CONTROLLER PIN CN3 - 9 (DO0)
const byte LENGTH_SENSOR = CONTROLLINO_A1; // LAM MOTOR CONTROLLER PIN CN3
const byte DOOR_SENSOR_FRONT = CONTROLLINO_A2;
const byte DOOR_SENSOR_REAR = CONTROLLINO_A3;

// INPUT INTERRUPT PINS
const byte ANGLE_SENSOR = CONTROLLINO_IN0; // --> INTERRUPT PIN

// RUNTIME
unsigned long runtime_start;

// FUNCTIONS *******************************************************************

void measure_runtime() {

  long number_of_cycles_to_measure = 1000;
  static long number_of_cycles_measured = 0;
  static unsigned long time_elapsed_this_one = 0;
  static unsigned long time_elapsed_all = 0;
  static unsigned long max_runtime = 0;
  static unsigned long min_runtime = 9999;

  time_elapsed_this_one = micros() - runtime_start;

  time_elapsed_all += time_elapsed_this_one;

  if (time_elapsed_this_one > max_runtime) {
    max_runtime = time_elapsed_this_one;
  }

  if (time_elapsed_this_one < min_runtime) {
    min_runtime = time_elapsed_this_one;
  }

  number_of_cycles_measured++;

  if (number_of_cycles_measured == number_of_cycles_to_measure) {

    float avg_runtime = float(time_elapsed_all) / number_of_cycles_to_measure;

    Serial.println();
    Serial.print("NUMBER OF CYCLES MEASURED: ");
    Serial.println(number_of_cycles_to_measure);

    Serial.print("TOTAL RUNTIME [ms]: ");
    Serial.println(time_elapsed_all / 1000);

    Serial.print("MIN RUNTIME [us]: ");
    Serial.println(min_runtime);

    Serial.print("AVG RUNTIME [us]: ");
    Serial.println(avg_runtime);

    Serial.print("MAX RUNTIME [us]: ");
    Serial.println(max_runtime);

    delay(1000);

    // RESET VALUES
    number_of_cycles_measured = 0;
    runtime_start = 0;
    min_runtime = 9999;
    max_runtime = 0;
    time_elapsed_all = 0;
  }
}

bool safety_door_is_closed() {
  bool safety_door_closed = false;
  if (digitalRead(DOOR_SENSOR_FRONT) && digitalRead(DOOR_SENSOR_REAR)) {
    safety_door_closed = true;
  }
  return safety_door_closed;
}

bool length_sensor_has_detected() {
  bool sensor_detected = false;
  if (digitalRead(LENGTH_SENSOR) == LOW) {
    sensor_detected = true;
  }
  return sensor_detected;
}

void set_motor_position_flag() //
{
  digitalWrite(MOTOR_SET_NEW_POSITION, HIGH);
}

void reset_motor_position_flag() //
{
  digitalWrite(MOTOR_SET_NEW_POSITION, LOW);
}

void enable_motor() //
{
  digitalWrite(MOTOR_ENABLE, HIGH);
}

void disable_motor() //
{
  digitalWrite(MOTOR_ENABLE, LOW);
}

// INTERRUPT FOR ANGLE CALIBRATION *********************************************
// RUNS EVERY TIME THE ANGLE SENSOR DETECTS A TURN

void isr_calibrate_angle() {
  if (length_sensor_has_detected()) {
    set_motor_position_flag();
  }
}

// SETUP ***********************************************************************

void setup() {

  Serial.begin(115200);

  // INTERRUPT FOR ANGLE SENSOR
  pinMode(ANGLE_SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANGLE_SENSOR), isr_calibrate_angle, RISING);

  // OUTPUT PINS
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_SET_NEW_POSITION, OUTPUT);

  Serial.println("EXIT SETUP");

  while(safety_door_is_closed()){
    // wait for user to open door
    // this prevents motor start during reset or startup
  }
}

// LOOP ************************************************************************

void loop() {

  runtime_start = micros();

  enum stage_names { DOOR_OPEN, DOOR_CLOSED };
  static int stage = DOOR_OPEN;

  switch (stage) {

  case DOOR_OPEN:
    disable_motor();

    if (length_sensor_has_detected() && safety_door_is_closed()) {
      Serial.println("CANNOT START, SLEDGE IS NOT IN START POSITION!");
      delay(500);
    }

    if (!length_sensor_has_detected() && safety_door_is_closed()) {
      Serial.println("DOOR_CLOSED");
      enable_motor();
      stage = DOOR_CLOSED;
    }
    break;

  case DOOR_CLOSED:

    // ISR IS TELLING MOTOR CONTROLLER WHEN TO START DECELERATION

    // USER HAS TO OPEN DOOR TO RESET MACHINE:
    if (!safety_door_is_closed()) {
      disable_motor();
      reset_motor_position_flag();
      Serial.println("DOOR OPENED");
      delay(500);
      stage = DOOR_OPEN;
    }
    break;

  default:
    break;
  }

  // measure_runtime(); // deactivate in production
}

// ********************************************************** END OF PROGRAM ***