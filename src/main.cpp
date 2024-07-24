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
 * TODO: •) After some time of inactivity, the Motor d
 * 
 * TODO: •) DOOR opens early -> DRIVER stops MOTOR as fast as possible!
 * 
 * TODO: •) LENGTH sensor detects already when user closes door
 * -> Spring already compressed, MOTOR must not start to turn
 * 
 * TODO: •) LENGTH sensor or Angle Sensor does not detect normally
 * -> use timeout
 * -> Mechanical or electrical malfungction, stop motor.
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
 * • ALL
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
const byte MOTOR_ENABLE = CONTROLLINO_D0; // -------- LAM MOTOR CONTROLLER PIN CN3 - 1
const byte MOTOR_STOP_PRECISE = CONTROLLINO_D1; // -- LAM MOTOR CONTROLLER PIN CN3 - 3
const byte MOTOR_EMERGENCY_STOP = CONTROLLINO_D3;// - LAM MOTOR CONTROLLER PIN CN3 - 5

// INPUT PINS
const byte MOTOR_STOPPED = CONTROLLINO_A7;// -------- LAM MOTOR CONTROLLER PIN CN3 - 9
const byte ANGLE_SENSOR = CONTROLLINO_A0;
const byte LENGTH_SENSOR = CONTROLLINO_A1;// LAM MOTOR CONTROLLER PIN CN3
Debounce door_sensor_front(CONTROLLINO_A2);
Debounce door_sensor_rear (CONTROLLINO_A3);

// INTERRUPT FOR ANGLE SENSOR
volatile bool angle_calibrated = false;
volatile unsigned int current_microstep_angle;

// RUNTIME
unsigned long runtime_start;

// PROGRAM STAGE BOOLS
bool sledge_is_in_start_position = true;
bool safety_door_closed = false;
bool endposition_calculated = false;
bool deceleration_initiated = false;
bool motor_stopped = false;

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

    delay(5000);

    // RESET VALUES
    number_of_cycles_measured = 0;
    runtime_start = 0;
    min_runtime = 9999;
    max_runtime = 0;
    time_elapsed_all = 0;
  }
}

void monitor_sensors() {
  if (door_sensor.switched_high()) {
    safety_door_closed = true;
  }
  if (door_sensor.switched_low()) {
    safety_door_closed = false;
  }
  if (length_sensor.switched_high()) {
    sledge_is_in_start_position = false;
  }
  if (length_sensor.switched_low()) {
    sledge_is_in_start_position = true;
  }
}


void run_motor() {
  }

void stop_motor() {
 
}

// INTERRUPT FOR ANGLE CALIBRATION *********************************************

void isr_calibrate_angle() {
  if (!angle_calibrated) {
    current_microstep_angle = 0;
    angle_calibrated = true;
  }
}

// SETUP ***********************************************************************

void setup() {

  Serial.begin(115200);

  // INTERRUPT FOR ANGLE SENSOR
  pinMode(CONTROLLINO_IN0, INPUT);
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN0), isr_calibrate_angle, RISING);

  pinMode(MOTOR_STEP_PIN, OUTPUT);

  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************

void loop() {

  runtime_start = micros();

  monitor_sensors();

  static int stage = 0;

  enum stage_names { READY, ACCELERATE, BRAKE, STOPPED, RESET };

  switch (stage) {

  case READY:
    if (safety_door_closed && sledge_is_in_start_position) {
      stage = ACCELERATE;
    }
    break;

  case ACCELERATE:
    run_motor();

    if (!safety_door_closed) {
      stage = BRAKE;
    }
    if (angle_calibrated && !sledge_is_in_start_position && !endposition_calculated) {
     
    } else if (endposition_calculated && !deceleration_initiated) {
      // is_looking_for_brakepoint = true;
    }
    if (deceleration_initiated) {
      stage = BRAKE;
    }
    break;

  case BRAKE:
    stop_motor();
    if (motor_stopped) {
      // generate_stepper_signal = false;
      stage = STOPPED;
    }
    break;

  case STOPPED:
    if (!safety_door_closed) {
      stage = RESET;
    }
    break;

  case RESET:
    deceleration_initiated = false;
    motor_stopped = false;
    stage = READY;
    Serial.println("RESET COMPLETED, READY FOR NEXT RUN");
    break;

  default:
    break;
  }
  // measure_runtime();

  // if (print_delay.delay_time_is_up(500)) {
  //   Serial.print("CURRENT ANGLE: ");
  //   Serial.print(current_microstep_angle);
  //   Serial.print(" | STEPS TO GO: ");
  //   Serial.print(remaining_steps_to_go);
  //   Serial.println("");
  // }
}

// ********************************************************** END OF PROGRAM ***