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
 * The maximum possible turns on the thread are 23.5, if the motor turns more often
 * the parts crash mechanically
 * 
 * The desired number of turns for the correct spring length of 40.5 are:
 *  11.5 turns
 * 
 * The steper driver is therefore limited to make maximum 16 Turns
 * 16 Turns x 128 microsteps per step x 200 steps per turn = Max position 409600
 * 
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
 * •) PLC enables informs DRIVER to enabble and accelerate MOTOR
 * 
 * •) PLC detects LENGTH -> PLC waits for next ANGLE
 *
 * •) PLC detects ANGLE -> DRIVER calculates and sets target position
 *    -> keep in loop calculations as slim as possible
 *
 * •) DRIVER stops MOTOR at target position
 * 
 * •) DRIVER informs PLC that MOTOR stopped
 * 
 * •) PLC disables MOTOR
 *
 * •) DRIVER resets
 * 
 * •) USER opens DOOR -> PLC resets
 * 
 * --------------  
 * SPECIAL EVENTS
 * --------------
 * 
 * •) DOOR opens while motor is running -> PLC sends emergency stop signal DRIVER!
 * 
 * •) LENGTH sensor detects already when user closes door
 * -> Spring already compressed, MOTOR must not start to turn
 * 
 * •) MOTOR runs to long
 * -> Mechanical or electrical malfungction, stop motor.
 * -> use timeout
 * 
 * *****************************************************************************
 * RUNTIME ON CONTROLLINO MAXI
 * 2023-08-23 MIN44 AVG48 MAX68 [us]
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

int max_motor_runtime = 5000; // [ms]
Insomnia motor_timeout(max_motor_runtime);

// GLOBAL VARIABLES ------------------------------------------------------------

// I/O-PINS:
// INPUTS:

// OUTPUT PINS:
const byte MOTOR_ENABLE = CONTROLLINO_D0; // --------- LAM MOTOR CONTROLLER PIN CN3 - PIN1 (DI0)
const byte MOTOR_STOP_PRECISE = CONTROLLINO_D1; // --- LAM MOTOR CONTROLLER PIN CN3 - PIN3 (DI1)
const byte MOTOR_EMERGENCY_STOP = CONTROLLINO_D2; // - LAM MOTOR CONTROLLER PIN CN3 - PIN5 (DI2)
const byte MOTOR_START = CONTROLLINO_D3; // ---------- LAM MOTOR CONTROLLER PIN CN3 - PIN7 (DI3)

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

bool safety_door_is_closed() {
  bool safety_door_closed = false;
  if (digitalRead(DOOR_SENSOR_FRONT) && digitalRead(DOOR_SENSOR_REAR)) {
    safety_door_closed = true;
  }
  return safety_door_closed;
}

bool emergency_stop_is_active() {
  bool is_active = false;
  if (digitalRead(MOTOR_EMERGENCY_STOP) == LOW) {
    is_active = true;
  }
  return is_active;
}

bool motor_is_turning() {
  bool is_turning = true;

  if (digitalRead(MOTOR_STOPPED) == HIGH) {
    is_turning = false;
  }
  return is_turning;
}

bool length_sensor_has_detected() {
  bool sensor_detected = false;
  if (digitalRead(LENGTH_SENSOR) == LOW) {
    sensor_detected = true;
  }
  return sensor_detected;
}

bool motor_stop_flag_is_set() {
  bool isr_detected = false;
  if (digitalRead(MOTOR_STOP_PRECISE)) {
    isr_detected = true;
  }
  return isr_detected;
}

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

    delay(5000);

    // RESET VALUES
    number_of_cycles_measured = 0;
    runtime_start = 0;
    min_runtime = 9999;
    max_runtime = 0;
    time_elapsed_all = 0;
  }
}

void set_motor_stop_flag() //
{
  digitalWrite(MOTOR_STOP_PRECISE, HIGH);
}

void reset_motor_stop_flag() //
{
  digitalWrite(MOTOR_STOP_PRECISE, LOW);
}

void enable_motor() //
{
  digitalWrite(MOTOR_ENABLE, HIGH);
}

void disable_motor() //
{
  //digitalWrite(MOTOR_ENABLE, LOW);
}

void initiate_emergency_stop() {
  enable_motor();
  digitalWrite(MOTOR_EMERGENCY_STOP, LOW);
}

void disable_emergency_stop() //
{
  digitalWrite(MOTOR_EMERGENCY_STOP, HIGH);
}

void start_motor() //
{
  digitalWrite(MOTOR_START, HIGH);
}

void disable_start_motor_signal() //
{
  digitalWrite(MOTOR_START, LOW);
}

// INTERRUPT FOR ANGLE CALIBRATION *********************************************
// THE ISR RUNS EVERY TIME THE ANGLE SENSOR DETECTS A TURN.
void isr_calibrate_angle() {
  if (length_sensor_has_detected()) {
    set_motor_stop_flag();
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
  pinMode(MOTOR_STOP_PRECISE, OUTPUT);
  pinMode(MOTOR_EMERGENCY_STOP, OUTPUT);

  // WAIT FOR THE MOTOR STOP POSITION OF THE MOTOR CONTROLLER

  Serial.println("WAITING FOR MOTOR TO STOP");
  // while (!motor_is_turning()) //
  // {
  // };

  reset_motor_stop_flag();
  disable_emergency_stop();

  // REMOVE THIS LATER ON:
  enable_motor();

  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************

void loop() {

  runtime_start = micros();

  enum stage_names { READY, ACCELERATE, BRAKE_IF_ISR, STOPPED, EMERGENCY_STOP, RESET };
  static int stage = STOPPED;

  // IF SAFETY DOOR IS NOT CLOSED !!!
  // STOP MOTOR IMMEDIATELY       !!!
  if (!safety_door_is_closed() && motor_is_turning()) {
    stage = EMERGENCY_STOP;
    if (!emergency_stop_is_active()) { // print only once
      Serial.println("DOOR - EMERGENCY STOP !!!");
    }
  }

  // IF SAFETY DOOR IS NOT CLOSED AND MOTOR HAS STOPPED
  // RESET RIG       !!!
  if (!safety_door_is_closed() && !motor_is_turning()) {
    stage = RESET;
    if (!emergency_stop_is_active()) { // print only once
      Serial.println("DOOR OPEN - RESET");
    }
  }

  switch (stage) {

  case READY:
    disable_motor();

    if (length_sensor_has_detected() && safety_door_is_closed()) {
      Serial.println("CANNOT START, SLEDGE IS NOT IN START POSITION!");
    }

    if (!length_sensor_has_detected() && safety_door_is_closed()) {
      Serial.println("ACCELERATE! ... BRAKE IF ISR");
      stage = ACCELERATE;
    }
    motor_timeout.reset_time();
    break;

  case ACCELERATE:
    enable_motor();
    start_motor();
    motor_timeout.reset_time();

    stage = BRAKE_IF_ISR;

    break;

  case BRAKE_IF_ISR:

    // ISR IS TELLING MOTOR CONTROLLER WHEN TO START DECELERATION

    // if (motor_timeout.has_timed_out()) {
    //   Serial.println("TIMEOUT, EMERGENCY STOP!");
    //   stage = EMERGENCY_STOP;
    // }
    if (motor_stop_flag_is_set()) {
      Serial.println("ISR DETECTED");
    }

    if (!motor_is_turning()) {
      delay(1000);
      Serial.println("DECELERATION COMPLETED");
      stage = STOPPED;
    }
    break;

  case STOPPED:
    disable_motor();
    disable_start_motor_signal();

    if (!motor_is_turning()) {
      delay(1000);
      Serial.println("MOTOR STOPPED");
      stage = STOPPED;
    }

    // USER HAS TO OPEN DOOR TO RESET MACHINE:
    if (!safety_door_is_closed()) {
      Serial.println("DOOR OPENED - RESET MACHINE");
      stage = RESET;
    }

    break;

  case RESET:
    disable_motor();
    disable_start_motor_signal();
    disable_emergency_stop();
    reset_motor_stop_flag();
    delay(500);
    Serial.println("RESET COMPLETED, READY FOR NEXT RUN");
    stage = READY;
    delay(1000);
    break;

  case EMERGENCY_STOP:
    disable_start_motor_signal();
    enable_motor();
    initiate_emergency_stop();
    if (motor_is_turning()) {
      Serial.println("EMERGENCY STOP COMPLETE JUMP TO RESET");
      stage = RESET;
    };
    break;

  default:
    break;
  }

  // measure_runtime();

  // if (print_delay.delay_time_is_up(500)) {
  //   Serial.print("LOESCHMICHBALD: ");
  //   Serial.print(loeschmichbald);
  //   Serial.println("");
  // }
}

// ********************************************************** END OF PROGRAM ***