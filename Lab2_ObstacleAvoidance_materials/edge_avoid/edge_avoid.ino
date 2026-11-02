/* ***********************************************************
 * VL53L0X / EDGE DETECTION VARIABLES
 *************************************************************/
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
// 5V
//#define SDA 21 //BLUE
//#define SCL 22 //GREEN
//#define VL53L0X_I2C_ADDR 0x29
#define TOF_L_ADDRESS 0x30
#define TOF_R_ADDRESS 0x31
#define TOF_O_ADDRESS 0x32

// set the pins to shutdown
#define SHT_L 23
#define SHT_R 15
#define SHT_O 19

// objects for the vl53l0x
Adafruit_VL53L0X left_TOF; // 
Adafruit_VL53L0X right_TOF;//
Adafruit_VL53L0X obst_TOF; // 

Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config = Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT;
// VL53L0X_SENSE_HIGH_ACCURACY   VL53L0X_SENSE_LONG_RANGE   VL53L0X_SENSE_DEFAULT VL53L0X_SENSE_HIGH_SPEED
// high speed too inaacurate. high acc too slow

const int EDGE_DISTANCE = 60; // sensor are 35-40mm away
const unsigned long EDGE_STOP_TIME = 250;
const unsigned long POST_TURN_STOP_TIME = 100;
const int TURN_TIME = 900;

enum EdgeState {
  EDGE_MONITORING,
  EDGE_TURNING,
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
#define POS_INC   10
#define POS_ANG   40
#define INTERVAL  40 // 20ms is fastest, <=100 bad
Servo servo;
int pos = 90; // 0-180, start at 90deg aka forwards
bool isCW = true;
unsigned long servoPrevMillis = 0;

// ULTRASONIC
// 5V
#define TRIG_PIN 4 // yellow
#define ECHO_PIN 2 // green
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
// - MotorA is LEFT | MotorB is RIGHT 
// Motor A connections
#define MOTOR_A_IN1 27 // AIN1
#define MOTOR_A_IN2 14 // AIN2
  
// Motor B connections
#define MOTOR_B_IN3 26 // BIN1
#define MOTOR_B_IN4 25 // BIN2

// PWM properties: channels 0-7 = high speed, 8-15 = low speed
#define PWM_RES     8 
#define PWM_FREQ    20000
#define PWM_CH_A1   12   
#define PWM_CH_A2   13 
#define PWM_CH_B1   14    
#define PWM_CH_B2   15 
 
const int SPEED_MAX = 220; // duty cycle = 0 - 255. @ 5v this is good
const int SPEED_MIN = 185; // 
int speed_now = SPEED_MIN; // 180 isnt enough for A/L
int turn_speed = SPEED_MIN ;  

void setup_obsAvoidance(){
  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);

  // set servo input pin
  servo.attach(SERVO_PIN);
  // set it to face forward/centre/90deg
  servo.write(pos);
  Serial.println(F("Pin is at {pos}"));
  //delay(2000);

  Serial.println("Obstacle Avoidance sensors are set up!");

}

void setup_tof(){
  pinMode(SHT_L, OUTPUT);
  pinMode(SHT_R, OUTPUT);
  pinMode(SHT_O, OUTPUT);

  Serial.println(F("Shutdown pins initiated..."));
  digitalWrite(SHT_L, LOW);
  digitalWrite(SHT_R, LOW);
  digitalWrite(SHT_O, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));

  // all reset
  digitalWrite(SHT_L, LOW);    
  digitalWrite(SHT_R, LOW);
  digitalWrite(SHT_O, LOW);
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

  // activating RIGHT
  digitalWrite(SHT_R, HIGH);
  //digitalWrite(SHT_L, LOW);    
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
  
  // start continuous ranging
  left_TOF.startRangeContinuous();
  right_TOF.startRangeContinuous();
  obst_TOF.startRangeContinuous(); 

  Serial.println("All 3 VL53L0X sensors are connected!"); 
}

void setup_motors(){
  // Set all the motor control pins to outputs
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Configure PWM for motor speed control using new API
  //bool ledcAttachChannel(uint8_t pin, uint32_t freq, uint8_t resolution, int8_t channel);
  ledcAttachChannel(MOTOR_A_IN1, PWM_FREQ, PWM_RES, PWM_CH_A1);
  ledcAttachChannel(MOTOR_A_IN2, PWM_FREQ, PWM_RES, PWM_CH_A2);
  ledcAttachChannel(MOTOR_B_IN3, PWM_FREQ, PWM_RES, PWM_CH_B1);
  ledcAttachChannel(MOTOR_B_IN4, PWM_FREQ, PWM_RES, PWM_CH_B2);

  // Turn off motors - Initial state
  motorA(0);
  motorB(0);

  Serial.println("Both motors are set up!");
}


void setup() {
  Serial.begin(115200);
  Serial.println("----- Starting Edge Detection RC !!! ----------------"); 
  delay(100); // allow me to switch to serial monitor

  /* ***********************************************************
   * MOTOR CONTROL SETUP
   *************************************************************/
  setup_motors();

  /* ***********************************************************
   * OBSTACLE AVOIDANCE SETUP
   *************************************************************/
  setup_obsAvoidance();// pos=90
  
  /************************************************************
   * VL53L0X / EDGE DETECTION SETUP
   *************************************************************/
  setup_tof(); 

  Serial.println("----- Wait 5 seconds...   ------- ----------------"); 
  delay(5000); // let me react
  
}


void loop() {
  unsigned long now = millis();

  if (edgeState == EDGE_TURNING) {
    if (now - edgeStateStartedAt < TURN_TIME) {
      Serial.print(" ^ currently turning!");
      return;
    }
    else{
      stop(); delay(100);
      Serial.print(" ^ Stopped turning!");
      edgeState = EDGE_MONITORING;
      edgeStateStartedAt = now;
      return;
    }
  }

  // === OBSTACLE AVOIDANCE LOGIC  ======================================================
  if (now - servoPrevMillis >= INTERVAL) {
    servoPrevMillis = now;

    if(pos < (90+POS_ANG) && isCW){
      pos += POS_INC;
      servo.write(pos);
    } else if (pos == (90+POS_ANG) && isCW){
      isCW = false;
      pos += POS_INC;
      servo.write(pos);
    } else if(pos > (90-POS_ANG) && !isCW){
      pos -= POS_INC;
      servo.write(pos);
    } else if(pos == (90-POS_ANG) && !isCW){
      isCW = true;
      pos -= POS_INC;
      servo.write(pos);
    }
  }
  Serial.print("pos is: "); Serial.print(pos);

  // get the distances measured by the obstacle detectors
  obst_d = getObstacleToFDistance(); 
  Serial.print(" | TOF: "); Serial.print(obst_d); Serial.print(" mm");
  ultrasonic_d = pingUltrasonic();
  Serial.print(" | USD: "); Serial.print(ultrasonic_d); Serial.println("mm");
  // === END OBSTACLE AVOIDANCE LOGIC  ==================================================
    
  // === EGDE DETECTION LOGIC ===========================================================
  

  // check that both EDGE TOFs are BOTH ready
  if (!readTOFs()) {
    return;
  }
  uint16_t left_d = left_TOF.readRangeResult();
  uint16_t right_d = right_TOF.readRangeResult();
  Serial.print("Right: ");  Serial.print(right_d);  
  Serial.print(" mm, Left: ");  Serial.print(left_d);  Serial.print(" mm ==> ");
  // === END EGDE DETECTION LOGIC =======================================================

  if (left_d <= EDGE_DISTANCE && right_d <= EDGE_DISTANCE &&
      obst_d > OBSTACLE_DISTANCE && ultrasonic_d > OBSTACLE_DISTANCE
  ) {
    moveForward(speed_now);
  } 
  else {
    Serial.print(" ! STOP ! : ");
    stop(); delay(EDGE_STOP_TIME);

    edgeState = EDGE_TURNING;
    edgeStateStartedAt = now;

    // edge ahead 
    if (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE){ 
      Serial.print("Edge Ahead! -> ");
      //turnRightForEdge = true;
      turnRight(speed_now);
    }
    // edge at left
    else if (left_d > EDGE_DISTANCE && right_d <= EDGE_DISTANCE){
      Serial.print("Edge LEFT! -> ");
      //turnRightForEdge = true;
      turnRight(speed_now);
    }
    //edge at right
    else if(left_d <= EDGE_DISTANCE && right_d > EDGE_DISTANCE){
      Serial.print("Edge RIGHT! -> ");
      //turnRightForEdge = false;
      turnLeft(speed_now);
    } 
    // object detected
    else if(obst_d <= OBSTACLE_DISTANCE || ultrasonic_d <= OBSTACLE_DISTANCE){
      Serial.print("OBJECT found at: "); Serial.print(pos);

      float distanceR = look(0); //700
      servo.write(90); delay(500);
      float distanceL = look(180); //700

      edgeStateStartedAt += 1900; // 700*2=1400 from looks + 500 after moving to 90

      Serial.print(" | left: "); Serial.print(distanceL); 
      Serial.print(" | right: "); Serial.println(distanceR); 

      if (distanceR >= distanceL)
        turnRight(speed_now);
      else
        turnLeft(speed_now);
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
float look(int angle){
  servo.write(angle); 
  delay(500);
  float US = pingUltrasonic();
  float tof = float(getObstacleToFDistance());
  float distance = min(US, tof );
  delay(200);
  return distance;
}

int getObstacleToFDistance(){
  int dist = obst_d;

  if(obst_TOF.isRangeComplete()) 
    dist = obst_TOF.readRangeResult();

  return dist;
}

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
  float distance_mm = 0.017 * duration_us * 10.0;

  return distance_mm;
}

void servoSweep(){
  if(pos < 135 && isCW){
    pos += POS_INC;
    //Serial.println("Servo turns CW");
  }
  else if (pos == 135 && isCW){
    isCW = false;
    pos += POS_INC;
  }
  else if(pos > 45 && !isCW){
    pos -= POS_INC;
    //Serial.println("Servo turns CCW");
  }
  else if(pos == 45 && !isCW){
    isCW = true;
    pos -= POS_INC;
  }
  Serial.print("pos is: "); Serial.println(pos);
  servo.write(pos);
}


/* ***********************************************************
  * MOTOR CONTROL - MOVEMENT FUNCTIONS
  *************************************************************/
// NOTE: 
// - MotorA is LEFT -> clockwise=forward, 
// - MotorB is RIGHT -> CounterCW=forward
// - both turns are on the spot i.e. rotate on its center

// Drive one channel with a signed speed:
//   speed > 0  -> forward (PWM on the first pin)
//   speed < 0  -> reverse (PWM on the second pin)
//   speed = 0  -> coast   (both pins LOW)
void driveChannel(int pinA, int pinB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(pinA, speed);
    ledcWrite(pinB, 0);
  } else if (speed < 0) {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, -speed);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, 0);
  }
}

void motorA(int speed) {
  driveChannel(MOTOR_A_IN1, MOTOR_A_IN2, speed);
}
void motorB(int speed) {
  driveChannel(MOTOR_B_IN3, MOTOR_B_IN4, speed);
}

void accelerate(int speedA, int speedB, int stepDelay) {
  int curA = 0, curB = 0;
  while(curA != speedA || curB != speedB) {
    if(curA < speedA) curA = min(curA + 5, speedA);
    if(curA > speedA) curA = max(curA - 5, speedA);
    if(curB < speedB) curB = min(curB + 5, speedB);
    if(curB > speedB) curB = max(curB - 5, speedB);
    motorA(curA);
    motorB(curB);
    delay(stepDelay);
  }
}

void moveForward(int speed){
  motorA(speed);
  motorB(speed);
  Serial.println("Moving Forward");
}
void moveBackwards(int speed){
  motorA(-speed);
  motorB(-speed);
  Serial.println("Moving Backwards");
}
void turnRight(int speed){
  // LEFT/A wheel forwards
  motorA(speed);
  // RIGHT/B wheel backwards
  motorB(-speed);
  Serial.println("Turning Right");
}
void turnLeft(int speed){
  // LEFT/A wheel backwards
  motorA(-speed);
  // RIGHT/B wheel forwards
  motorB(speed);
  Serial.println("Turning Left");
}
// Turn off motors
void stop() {
  motorA(0);
  motorB(0);
  Serial.println("Coast stopping");
}

void testMotorMovement(){
  moveForward(speed_now);
  delay(5000);
  moveBackwards(speed_now);
  delay(5000);
  turnRight(speed_now);
  delay(5000);
  turnLeft(speed_now);
  delay(5000);
  stop();
  delay(5000);
}
