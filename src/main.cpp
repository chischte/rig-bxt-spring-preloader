/*
 * *****************************************************************************
 * SPRING PRELOADER
 * *****************************************************************************
 * 
 * Michael Wettstein
 * August 2023, Zürich
 * 
 * *****************************************************************************
 * Program to control an assembly device that preloads a spring
 * using a screw joint
 * 
 * The spring must always be compressed to the same length and the machine
 * must always stop at the desired angle
 * 
 * *****************************************************************************
 * ELEMENTS
 * *****************************************************************************
 * 
 * PLC ----> Controllino Maxi PLC ----> monitors sensors for safety and position
 * DRIVER -> LAM DS3078 Motor Driver -> accelerates and decelerates stepper motor
 * MOTOR --> NEMA34 Stepper ----------> turns screw joint to compress spring to 
 *                                      desired length and angle
 * DOOR ---> Sensor to detect if safety door is closed
 * LENGTH -> Sensor to detect if thread is at a certain length
 * ANGLE --> Sensor to detect a fixed angle at every motor turn
 * 
 * *****************************************************************************
 * PROGRAM STRUCTURE
 * *****************************************************************************
 * ----------------
 * NORMAL OPERATION
 * ----------------
 * 0) USER CLOSES DOOR
 * 
 * 1) PLC checks that DOOR is closed and LENGTH does not detect
 *    -> Thread is in start position and the spring not preloaded yet
 * 
 * 2) PLC informs DRIVER to accelerate MOTOR
 * 
 * 3) PLC detects LENGTH -> PLC waits for next ANGLE
 *
 * 4) PLC detects ANGLE -> DRIVER calculates and sets target position
 *
 * 5) DRIVER stops MOTOR at target position
 *
 * 6) DRIVER resets
 * 
 * 7) USER opens DOOR -> PLC resets
 * 
 * --------------  
 * SPECIAL EVENTS
 * --------------
 * 
 * A) DOOR opens early -> DRIVER stops MOTOR as fast as possible!
 * 
 * B) LENGTH sensor detects already when user closes door
 *    -> MOTOR must not start to turn
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
 * • Implement second door sensor
 * • Implement Motor driver
 * *****************************************************************************
 * 
 * 
 * 
 * 
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
Debounce angle_sensor(CONTROLLINO_A0);
Debounce length_sensor(CONTROLLINO_A1);
Debounce door_sensor(CONTROLLINO_A2);

// OUTPUTS:
const byte MOTOR_STEP_PIN = CONTROLLINO_D3;

// INTERRUPT FOR STEPPER SIGNAL GENERATION
volatile bool generate_stepper_signal = false;
volatile bool step_is_completed = false;
volatile bool is_looking_for_brakepoint = false;
volatile bool update_speedlevel = false;
volatile unsigned long remaining_steps_to_go;
volatile unsigned int current_speed_level;
volatile int microsteps_per_turn;
float interrupt_period; //[us]

// INTERRUPT FOR ANGLE SENSOR
volatile bool angle_calibrated = false;
volatile unsigned int current_microstep_angle;

// RUNTIME
unsigned long runtime_start;

// PRELOAD SETTINGS
float add_preload_full_turns = 5; //            <<< ADJUST PRELOAD POSITION HERE
float add_preload_degrees = 12; // [°] ≥0       <<< ADJUST PRELOAD POSITION HERE
unsigned long add_preload_microsteps;

// MOTOR PARAMETERS
float acceleration = 1200; // [rpm/s]                <<< ADJUST ACCELERATION HERE
float topspeed = 700; // [rpm]                          <<< ADJUST TOPSPEED HERE
const int full_steps_per_turn = 200; // 360/1.8°
const int micro_step_factor = 2;

// ACCELERATION RAMP ARRAY
unsigned int step_delay_array[3333]; // make big enough but do not overload RAM
unsigned int highest_speed_level;

// SPEED AND POSITION VALUES

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

void calculate_remaining_steps_to_go() {
  unsigned long steps_till_zero = microsteps_per_turn - current_microstep_angle;
  remaining_steps_to_go = steps_till_zero + add_preload_microsteps;

  // STEPS TO GO MUST BE AT LEAST CURRENT SPEEDLEVEL
  // (C)URRENT SPEEDLEVEL = MINIMUM BRAKETIME)
  while (remaining_steps_to_go < current_speed_level) {
    remaining_steps_to_go += microsteps_per_turn;
  }

  endposition_calculated = true;
}

void increase_current_speed_level() {
  current_speed_level++;
  if (current_speed_level >= highest_speed_level) {
    current_speed_level = highest_speed_level;
  }
}

void decrease_current_speed_level() {
  current_speed_level--;
  if (current_speed_level == 0) {
    motor_stopped = true;
  }
}

void update_interrupt_timer() {
  unsigned long current_step_delay = step_delay_array[current_speed_level];
  float current_switch_delay = float(current_step_delay) / 2;
  unsigned int current_compare_match_register_value = int(current_switch_delay / interrupt_period);

  OCR1A = current_compare_match_register_value;
}

void run_motor() {
  if (update_speedlevel) {
    increase_current_speed_level();
    update_interrupt_timer();
    update_speedlevel = false;
  }
}

void stop_motor() {
  if (update_speedlevel) {
    decrease_current_speed_level();
    update_interrupt_timer();
    update_speedlevel = false;
  }
}

// INITIAL CALCULATIONS --------------------------------------------------------

void make_initial_calculations() {

  float prev_endtime = 0;

  Serial.println("");

  microsteps_per_turn = full_steps_per_turn * micro_step_factor;

  float turns_per_switch = 1 / float(microsteps_per_turn);
  float acceleration_time = topspeed / acceleration;
  float turns_to_topspeed = acceleration_time * (topspeed / 60) / 2;
  float delay_at_topspeed = 1000.0 * 1000.0 / (topspeed / 60 * microsteps_per_turn); // [us]
  float add_preload_turns = add_preload_full_turns + add_preload_degrees / 365.0;
  add_preload_microsteps = add_preload_turns * microsteps_per_turn;

  bool topspeed_calculated = false;

  while (!topspeed_calculated) {

    static int current_step = 0;
    static int printcounter = 0;

    float endposition = turns_per_switch * (float(current_step) + 1.0);
    float endtime = sqrt(2 * endposition / (acceleration / 60)) * 1000;
    float delay = (endtime - prev_endtime) * 1000.0;
    prev_endtime = endtime;
    float stepspeed = endtime / 1000 * acceleration;

    step_delay_array[current_step] = int(delay);

    topspeed_calculated = stepspeed >= topspeed;
    if (topspeed_calculated) {
      highest_speed_level = current_step;
    }

    if ((printcounter == 0) | topspeed_calculated) {

      Serial.print("STEP ");
      Serial.print(current_step);
      Serial.print(" | ");

      Serial.print(" ENDPOSITION: ");
      Serial.print(endposition, 4);
      Serial.print(" [turns] | ");

      Serial.print("ENDTIME: ");
      Serial.print(endtime, 0);
      Serial.print(" [ms] | ");

      Serial.print("DELAY: ");
      Serial.print(int(delay));
      Serial.print(" [us] | ");
      prev_endtime = endtime;

      Serial.print("ENDSPEED: ");
      Serial.print(stepspeed, 0);
      Serial.print(" [rpm] ");

      Serial.println("");

      printcounter = 50;
    }

    current_step++;
    printcounter--;
  }

  Serial.println("******************************************************");
  Serial.print("TIME TO REACH TOPSPEED: ");
  Serial.print(acceleration_time, 1);
  Serial.println(" s");

  Serial.print("NUMBERS OF TURNS TO REACH TOPSPEED: ");
  Serial.println(turns_to_topspeed, 1);

  Serial.print("DELAY AT TOPSPEED: ");
  Serial.print(delay_at_topspeed, 0);
  Serial.println(" us");

  Serial.print("HIGHEST STEP NUMBER: ");
  Serial.print(highest_speed_level);
  Serial.println("");

  Serial.println("SPRING PRELOAD PARAMETERS ****************************");

  Serial.print("NUMBER OF ADDITIONAL TURNS: ");
  Serial.print(add_preload_full_turns);
  Serial.println("");

  Serial.print("NUMBER OF ADDITIONAL DEGREES: ");
  Serial.print(add_preload_degrees);
  Serial.println("");

  Serial.print("NUMBER OF ADDITIONAL MICROSTEPS: ");
  Serial.print(add_preload_microsteps);
  Serial.println("");

  Serial.println("******************************************************");
  Serial.print("INTERRUPT PERIOD: ");
  Serial.print(interrupt_period, 1);
  Serial.println(" us");
}

// INTERRUPT FOR ANGLE CALIBRATION *********************************************

void isr_calibrate_angle() {
  if (!angle_calibrated) {
    current_microstep_angle = 0;
    angle_calibrated = true;
  }
}

// INTERRUPT TIMER FOR STEPPER MOTOR********************************************

void toggle_step_pin() {

  // STEP PIN CONTROLLINO: Digital 3
  // PORT PIN: PE3
  // WRITE A BIT WITH 1 TOGGLES IT

  PINE = PINE | 0b00001000;
  // digitalWrite(MOTOR_STEP_PIN, !digitalRead(MOTOR_STEP_PIN));
}

void interrupt_setup() {
  cli(); //stop interrupts

  // CLEAR REGISTERS OF TIMER 1
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0; //initialize counter value to 0

  // SET COMPARE MATCH VALUE (0-65565)
  OCR1A = 5000;

  // Turn on CTC mode (Clear Timer On Compare Match)
  TCCR1B |= (1 << WGM12);

  // SET PRESCALER TO  8 -> CS12 - CS10 = 010
  // SET PRESCALER TO 64 -> CS12 - CS10 = 011
  TCCR1B |= (0 << CS12) | 1 << CS11 | (0 << CS10);

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //allow interrupts

  float arduino_frequency = 16.0; // [MHz]
  float arduino_clock_cycle = 1.0 / arduino_frequency; // [us]
  float prescaler = 8.0;
  interrupt_period = arduino_clock_cycle * prescaler;
}

ISR(TIMER1_COMPA_vect) {

  static volatile int step_stage = 0; // 0=fresh / 1 switched on / 2 switched off

  if (generate_stepper_signal) {
    toggle_step_pin();
    step_stage++;
    if (step_stage == 2) {
      step_is_completed = true;
      step_stage = 0;
    }
  }

  if (step_is_completed) {

    step_is_completed = false;
    update_speedlevel = true;

    remaining_steps_to_go--;
    current_microstep_angle++;

    if (current_microstep_angle == microsteps_per_turn) {
      current_microstep_angle = 0;
    }

    if (is_looking_for_brakepoint) {
      if (current_speed_level == remaining_steps_to_go) {
        deceleration_initiated = true;
      }
    }
  }
}

// SETUP ***********************************************************************

void setup() {

  Serial.begin(115200);

  // INTERRUPT FOR STEPPER SIGNAL
  interrupt_setup();

  // INTERRUPT FOR ANGLE SENSOR
  pinMode(CONTROLLINO_IN0, INPUT);
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN0), isr_calibrate_angle, RISING);

  make_initial_calculations();

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
      generate_stepper_signal = true;
      stage = ACCELERATE;
    }
    break;

  case ACCELERATE:
    run_motor();

    if (!safety_door_closed) {
      stage = BRAKE;
    }
    if (angle_calibrated && !sledge_is_in_start_position && !endposition_calculated) {
      calculate_remaining_steps_to_go();
    } else if (endposition_calculated && !deceleration_initiated) {
      is_looking_for_brakepoint = true;
    }
    if (deceleration_initiated) {
      stage = BRAKE;
    }
    break;

  case BRAKE:
    stop_motor();
    if (motor_stopped) {
      generate_stepper_signal = false;
      stage = STOPPED;
    }
    break;

  case STOPPED:
    if (!safety_door_closed) {
      stage = RESET;
    }
    break;

  case RESET:
    angle_calibrated = false;
    sledge_is_in_start_position = true;
    endposition_calculated = false;
    deceleration_initiated = false;
    is_looking_for_brakepoint = false;
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