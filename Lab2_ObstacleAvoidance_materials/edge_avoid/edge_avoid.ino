/* ***********************************************************
 * VL53L0X / EDGE DETECTION VARIABLES
 *************************************************************/
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
// 5V
#define SDA 21 
#define SCL 22

#define TOF_L_ADDRESS 0x30
#define TOF_R_ADDRESS 0x31
#define TOF_O_ADDRESS 0x32

// set the pins to shutdown
#define SHT_L 23
#define SHT_R 15
#define SHT_O 4

// objects for the vl53l0x
Adafruit_VL53L0X left_TOF; // 
Adafruit_VL53L0X right_TOF;//
Adafruit_VL53L0X obst_TOF; // 

//Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config = Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT;
// VL53L0X_SENSE_HIGH_ACCURACY   VL53L0X_SENSE_LONG_RANGE   VL53L0X_SENSE_DEFAULT VL53L0X_SENSE_HIGH_SPEED
// high speed too inaacurate. high acc too slow

const int EDGE_DISTANCE = 70; // sensor are 35-40mm away
const unsigned long EDGE_STOP_TIME = 300;
const unsigned long POST_TURN_STOP_TIME = 100;
const int TURN_TIME = 900;

enum EdgeState {
  EDGE_MONITORING,
  //EDGE_STOPPING_BEFORE_TURN,
  EDGE_TURNING,
  //EDGE_STOPPING_AFTER_TURN
};
EdgeState edgeState = EDGE_MONITORING;
unsigned long edgeStateStartedAt;
bool turnRightForEdge;

/* ***********************************************************
 * OBSTACLE AVOIDANCE VARIABLES
 *************************************************************/
// SERVO MOTOR
#include <ESP32Servo.h>
//5v
#define SERVO_PIN 13
Servo servo;
int pos = 90; // 0-180, start at 90deg aka forwards
bool isCW = true;

// ULTRASONIC
// 5V
#define TRIG_PIN 18
#define ECHO_PIN 5 
float ultrasonic_d;

#define OBSTACLE_DISTANCE 200 // apply both for TOF and ultrasonic
enum ObstacleState {
  OBSTACLE_MONITORING,
  OBSTACLE_TURNING,
};
ObstacleState obstacleState = OBSTACLE_MONITORING;
unsigned long obsStateStartedAt;
bool turnRightForObstacle;

uint16_t obst_d = 0;



/* ***********************************************************
 * MOTOR CONTROL VARIABLES
 *************************************************************/
// - MotorA is right | MotorB is left 
// Motor A connections
#define EN_A_R       14  // EN_A_R pin
#define MOTOR_A_IN_1 27  // IN1 pin
#define MOTOR_A_IN_2 26  // IN2 pin

// Motor B connections
#define MOTOR_B_IN_3 25  // IN3 pin
#define MOTOR_B_IN_4 33  // IN4 pin
#define EN_B_L       32  // EN_B_L pin

// PWM properties
const int FREQ = 30000;      // PWM FREQuency
const int PWM_CHANNEL_A = 1; // PWM channel for motor A, 0-15
const int PWM_CHANNEL_B = 2; // PWM channel for motor B, 0-15
const int RESOLUTION = 8;    // 1-16bits. 8bit -> 0-255 duty cycle
const int MOTOR_SPEED_MAX = 205; // duty cycle = 0 - 255. @ 5v this is good
const int RIGHT_PLUS = MOTOR_SPEED_MAX + 5;
const int MOTOR_SPEED_TURN = MOTOR_SPEED_MAX - 5;   // testing shows both can go from min 130 @ 9V, but 5V needs more



void setup_obsAvoidance(){
  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);

  // set servo input pin
  servo.attach(SERVO_PIN);
  // set it to face forward/centre/90deg
  servo.write(pos);
}

void setup_tof(){
  pinMode(SHT_L, OUTPUT);
  pinMode(SHT_R, OUTPUT);
  pinMode(SHT_O, OUTPUT);

  Serial.println(F("Shutdown pins initiated..."));
  digitalWrite(SHT_L, LOW);
  digitalWrite(SHT_R, LOW);
  //digitalWrite(SHT_O, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));
/*
  // initialize 1 I2C buses for OBS using default 
  if (!obst_TOF.begin() ) {
    Serial.println("Could not find a valid VL53L0X_obs sensor, check wiring!");
    while (1);
  }
  Serial.println(F("Booted OBSTACLE VL53L0X successfully!"));
  delay(10);  */
  
  // all reset
  digitalWrite(SHT_L, LOW);    
  digitalWrite(SHT_R, LOW);
  digitalWrite(SHT_O, LOW);
  delay(10);

  // activating RIGHT
  digitalWrite(SHT_R, HIGH);
  delay(10);
  if(!right_TOF.begin(TOF_R_ADDRESS)) {
    Serial.println(F("Failed to boot RIGHT VL53L0X"));
    while(1);
  }
  Serial.println(F("Booted RIGHT VL53L0X successfully!"));
  delay(10);

  // activating LEFT
  digitalWrite(SHT_L, HIGH);
  delay(10);
  if(!left_TOF.begin(TOF_L_ADDRESS)) {
    Serial.println(F("Failed to boot LEFT VL53L0X"));
    while(1);
  }
  Serial.println(F("Booted LEFT VL53L0X successfully!"));
  delay(10);

   // activating OBS
  digitalWrite(SHT_O, HIGH);
  delay(10);
  if(!obst_TOF.begin(TOF_O_ADDRESS)) {
    Serial.println(F("Failed to boot OBSTACLE VL53L0X"));
    while(1);
  }
  Serial.println(F("Booted OBSTACLE VL53L0X successfully!"));
  delay(10);
  
  // start continuous ranging
  left_TOF.startRangeContinuous();
  right_TOF.startRangeContinuous();
  obst_TOF.startRangeContinuous(); 
}

void setup_motors(){
  // Set all the motor control pins to outputs
  pinMode(MOTOR_A_IN_1, OUTPUT);
  pinMode(MOTOR_A_IN_2, OUTPUT);
  pinMode(MOTOR_B_IN_3, OUTPUT);
  pinMode(MOTOR_B_IN_4, OUTPUT);
  
  // Configure PWM for motor speed control using new API
  ledcAttachChannel(EN_A_R, FREQ, RESOLUTION, PWM_CHANNEL_A);
  ledcAttachChannel(EN_B_L, FREQ, RESOLUTION, PWM_CHANNEL_B);
  
  // Turn off motors - Initial state
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, LOW);
  
  // Set initial PWM duty cycle to 0 (motors off)
  ledcWrite(EN_A_R, 0);
  ledcWrite(EN_B_L, 0);
}


void setup() {
  Serial.begin(115200);
  Serial.println("----- Starting Edge Detection RC !!! ----------------"); 
  delay(500); // allow me to switch to serial monitor

  /* ***********************************************************
   * MOTOR CONTROL SETUP
   *************************************************************/
  setup_motors();
  Serial.println("Both motors are set up!");

  /************************************************************
   * VL53L0X / EDGE DETECTION SETUP
   *************************************************************/
  setup_tof(); 
  Serial.println("Both edge VL53L0X sensors are connected!"); 

  /* ***********************************************************
   * OBSTACLE AVOIDANCE SETUP
   *************************************************************/
  setup_obsAvoidance();// pos=90
  Serial.println("Obstacle Avoidance sensors are set up!");


  Serial.println("----- Wait 3 seconds...   ------- ----------------"); 
  delay(5000); // let me react
}


void loop() {
  //testMotorMovement();
  //testTOFsensors();

  //testEdge();
  unsigned long now = millis();


  /*
  // === OBSTACLE AVOIDANCE LOGIC  ======================================================
  if(obstacleState == OBSTACLE_TURNING){
    obstacleState = OBSTACLE_MONITORING
  }
  */

  obst_d = obst_TOF.readRangeResult();
  obst_d = (obst_TOF.readRangeResult() + obst_d) / 2.0;
  Serial.print(" | TOF: "); Serial.print(obst_d); Serial.println(" m");

  ultrasonic_d = pingUltrasonic();
  ultrasonic_d = (pingUltrasonic() + ultrasonic_d) / 2.0;
  Serial.print(" | USD: "); Serial.print(ultrasonic_d); Serial.println(" mm");

  /*if(obst_d <= OBSTACLE_DISTANCE || ultrasonic_d <= OBSTACLE_DISTANCE){
    obstacleState = OBSTACLE_TURNING;
    obsStateStartedAt = now;
    // turnRightForObstacle
    stop(); delay(EDGE_STOP_TIME);
    
  // if < safe distance
  // - stop
  // - check 180, 150, 120,  90,  60,  30,   0
  //           L    L    L    F    R    R    R
  // - turn to longest side, L or R, for ~ 1/2 turn time
  }*/
  // === END OBSTACLE AVOIDANCE LOGIC  ==================================================


  if(pos < 140 && isCW){
    pos += 10;
    //Serial.println("Servo turns CW");
  }
  else if (pos == 140 && isCW){
    isCW = false;
    pos -= 10;
  }
  else if(pos > 40 && !isCW){
    pos -= 10;
    //Serial.println("Servo turns CCW");
  }
  else if(pos == 40 && !isCW){
    isCW = true;
    pos += 10;
  }
  Serial.print("pos is: "); Serial.println(pos);
  //Serial.print(" | time is: "); Serial.println(now);
  servo.write(pos);

  //}
  
  
    // === EGDE DETECTION LOGIC ===========================================================
  if (edgeState == EDGE_TURNING) {
    Serial.println(" ^ currently still turning!");
    if (now - edgeStateStartedAt < TURN_TIME) {
      return;
    }
    //stop(); delay(100);
    edgeState = EDGE_MONITORING;
    edgeStateStartedAt = now;
    return;
  }

  // check that both EDGE TOFs are BOTH ready
  if (!readTOFs()) {
    return;
  }

  uint16_t left_d = left_TOF.readRangeResult();
  uint16_t right_d = right_TOF.readRangeResult();
  Serial.print("Right: ");  Serial.print(right_d);  
  Serial.print(" mm, Left: ");  Serial.print(left_d);  Serial.print(" mm ==> ");
  // === END EGDE DETECTION LOGIC =======================================================


  if (left_d <= EDGE_DISTANCE && right_d <= EDGE_DISTANCE) {
    moveForward();
  } 
  else {
    Serial.print(" EDGE DETECTED! : ");
    stop(); delay(EDGE_STOP_TIME);

    edgeState = EDGE_TURNING;
    edgeStateStartedAt = now;

    // edge ahead 
    if (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE){ 
      Serial.print("Edge Ahead! -> ");
      //turnRightForEdge = true;
      turnRight();
    }
    // edge at left
    else if (left_d > EDGE_DISTANCE && right_d <= EDGE_DISTANCE){
      Serial.print("Edge LEFT! -> ");
      //turnRightForEdge = true;
      turnRight();
    }
    //edge at right
    else if(left_d <= EDGE_DISTANCE && right_d > EDGE_DISTANCE){
      Serial.print("Edge RIGHT! -> ");
      //turnRightForEdge = false;
      turnLeft();
    }
  }//stop edge turn ELSE

}// END LOOP


/* ***********************************************************
 * VL53L0X / EDGE DETECTION FUNCTIONS
 *************************************************************/
void testTOFsensors(){

  uint16_t left_d;
  uint16_t right_d;

  if ( !readTOFs() ){
    return;  
  }  
  Serial.print("RIGHT Distance in mm: ");
  Serial.print(right_TOF.readRangeResult());

  Serial.print(" | LEFT Distance in mm: ");
  Serial.println(left_TOF.readRangeResult());
  
}
/* There are 2 sensors to check
 * This functions handles checking that both TOFs are ready at the same time
 */
bool readTOFs() {
  if (!left_TOF.isRangeComplete() || !right_TOF.isRangeComplete() ) {
    return false;
  }
  return true;
}


/* ***********************************************************
 * OBSTACLE AVOIDANCE FUNCTIONS
 *************************************************************/
float pingUltrasonic(){
  // ping thr trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  float distance_mm = 0.017 * duration_us * 100;

  return distance_mm;
}

void servoSweep(){
  if(pos < 140 && isCW){
    pos += 1;
    //Serial.println("Servo turns CW");
  }
  else if (pos == 140 && isCW){
    isCW = false;
    pos += 1;
  }
  else if(pos > 40 && !isCW){
    pos -= 1;
    //Serial.println("Servo turns CCW");
  }
  else if(pos == 40 && !isCW){
    isCW = true;
    pos -= 1;
  }
  Serial.print("pos is: "); Serial.println(pos);
  servo.write(pos);
}


/* ***********************************************************
  * MOTOR CONTROL - MOVEMENT FUNCTIONS
  *************************************************************/
// NOTE: 
// - MotorA is right -> clockwise=forward, MotorB is left -> CounterCW=forward
//   => flip wires ( or set inverse IN values )
// - both turns are on the spot i.e. rotate on its center

void moveForward(){
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

  ledcWrite(EN_A_R, RIGHT_PLUS);
  ledcWrite(EN_B_L, MOTOR_SPEED_MAX);

  Serial.println("Moving Forward");
}
void moveBackwards(){
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A_R, RIGHT_PLUS);
  ledcWrite(EN_B_L, MOTOR_SPEED_MAX);

  Serial.println("Moving Backwards");

}
void turnRight(){
  // right/A wheel BACKWARDS
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  // left/B wheel FORWARDS
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

  ledcWrite(EN_A_R, MOTOR_SPEED_TURN+4);
  ledcWrite(EN_B_L, MOTOR_SPEED_TURN);

  Serial.println("Turning Right");
}
void turnLeft(){
  // right/A wheel forward
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  // left/B wheel backwards
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A_R, MOTOR_SPEED_TURN+4);
  ledcWrite(EN_B_L, MOTOR_SPEED_TURN);

  Serial.println("Turning Left");
}
// Turn off motors
void stop() {
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A_R, 0);
  ledcWrite(EN_B_L, 0);

  Serial.println("Motor stopped");
}

void slowDown( int dutyCycle ){
  while (dutyCycle >= MOTOR_SPEED_MAX-30){
    ledcWrite(EN_A_R, dutyCycle);
    ledcWrite(EN_B_L, dutyCycle);
    Serial.print(">> Speed is: "); Serial.println(dutyCycle);
    dutyCycle = dutyCycle - 5;
  }
}

void testMotorMovement(){
  moveForward();
  testTOFsensors();
  delay(7000);
  moveBackwards();
  testTOFsensors();
  delay(7000);
  turnRight();
  testTOFsensors();
  delay(10000);
  turnLeft();
  testTOFsensors();
  delay(10000);
  stop();
  testTOFsensors();
  delay(7000);
}
