/* ***********************************************************
 * LIBRARIES
 *************************************************************/
#include <Wire.h> // for vl530l0x
#include "Adafruit_VL53L0X.h" // for vl530l0x
#include <ESP32Servo.h> // for the servo
#include <IRremote.hpp> // IR detection

/* ***********************************************************
 * ESP32 PINS USED
 *************************************************************/
/* vl530l0x xshut pins - 3.3V */
#define XSHT_PIN_L  23
#define XSHT_PIN_R  18
#define XSHT_PIN_T  19
//#define SDA 21 // BLUE  / ORANGE
//#define SCL 22 // GREEN / YELLOW
/* ultrasonic's pins - 5V */
#define TRIG_PIN    4 // yellow
#define ECHO_PIN    2 // green

/* servo pin - 5V*/
#define SERVO_PIN   13
/* DRV8833 motor input/control pins */
#define MOTOR_A_IN1 27 // AIN1
#define MOTOR_A_IN2 14 // AIN2
#define MOTOR_B_IN3 26 // BIN1
#define MOTOR_B_IN4 25 // BIN2
/*IR obstacle detector pins - 3.3V*/
#define L_IR_ANALOG_PIN 34
#define R_IR_ANALOG_PIN 15
#define MR_IR_ANALOG_PIN 33
#define ML_IR_ANALOG_PIN 35
/*IR Light Detector - 3.3V*/
#define IR_DETECTOR_PIN 32

/* ***********************************************************
 * MOTOR VARIABLES
 *************************************************************/
// PWM properties: channels 0-7 = high speed, 8-15 = low speed
const int PWM_RES =   8; 
const int PWM_FREQ =  20000;
const int PWM_CH_A1 = 12;   
const int PWM_CH_A2 = 13; 
const int PWM_CH_B1 = 14;    
const int PWM_CH_B2 = 15;

// speed, turn, and stop
const unsigned long STOP_TIME = 250;
const unsigned long POST_TURN_STOP_TIME = 100;
const int TURN_TIME = 900;
const int TURN_90DEG_TIME = 800;
const int SPEED_MAX = 215; // line following turns
const int SPEED_MIN = 187; // 180 isnt enough for A/L. 185 has some issues
int speed_now = SPEED_MIN; // 

/* ***********************************************************
 * VL53L0X related variables
 *************************************************************/
// addresses
#define TOF_L_ADDRESS 0x30
#define TOF_R_ADDRESS 0x31
#define TOF_T_ADDRESS 0x32

// objects for the vl53l0x
Adafruit_VL53L0X left_TOF;  
Adafruit_VL53L0X right_TOF;
Adafruit_VL53L0X top_TOF; 

/* ***********************************************************
 * Object and Turn Variables
 *************************************************************/
int min_obs = 0;
const int OBSTACLE_DISTANCE = 275; // apply both for TOF and ultrasonic
enum ObjectState {
  OBJECT_MONITORING,
  OBJECT_TURNING,
};
ObjectState objectState = OBJECT_MONITORING;
unsigned long objectStateStartedAt;

// servo related
Servo servo;
int pos = 90; // 0-180, start at 90deg aka forwards
bool isCW = true; // to control sweeo direction
const int POS_INC = 18; // the dif between each stopped position
const int POS_ANG = 36; // +/- from 90deg
const int POS_INTERVAL = 70; //20ms is the fastest the servo can go
// 10, 40, 40 was previous values
unsigned long servoPrevMillis = 0; // used to time sweep increments

/* ***********************************************************
 * LINE FOLLOWING VARIABLES
 *************************************************************/
const int LEFT_LINE_THRESHOLD   = 300; // high 200 -> 1100s
const int MIDDLE_L_LINE_THRESHOLD = 300; // low 300s -> 3700s
const int MIDDLE_R_LINE_THRESHOLD = 500; // low 300s -> 3700s
const int RIGHT_LINE_THRESHOLD  = 1000; // high 200 -> 800s



int L_lineAnalog, R_lineAnalog, MR_lineAnalog, ML_lineAnalog;
bool lineAlreadyFound = false;
bool leftOnLine = false;
bool rightOnLine = false;
bool middleRightOnLine = false;
bool middleLeftOnLine = false;


unsigned long linePrevMillis = 0;
const int LINE_INTERVAL = 1000;

bool isLightOn = false;
unsigned long lightLastOnMillis = 0;

/* ***********************************************************
* SETUP functions
*************************************************************/
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

  Serial.println("! --- Both motors are set up! ---");
}

void setup_obsAvoidance(){
  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);

  // set servo input pin
  servo.attach(SERVO_PIN);
  //build up the capacitors a bit first
  for(int angle = 0; angle <=180; angle+=45){
    servo.write(angle); delay(500);
  }
  // set it to face forward/centre/90deg
  servo.write(pos); delay(500);
  Serial.print(F("Pin is at ")); Serial.println(pos);

  Serial.println("! --- Obstacle Avoidance sensors are set up! ---");
}

void setup_tof(){
  // configure xshut pins
  pinMode(XSHT_PIN_L, OUTPUT);
  pinMode(XSHT_PIN_R, OUTPUT);
  pinMode(XSHT_PIN_T, OUTPUT);

  // set xshuts low to indicate address rewrite
  Serial.println(F("Shutdown pins initiated..."));
  digitalWrite(XSHT_PIN_L, LOW);
  digitalWrite(XSHT_PIN_R, LOW);
  digitalWrite(XSHT_PIN_T, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));

  // all reset 
  digitalWrite(XSHT_PIN_L, LOW);    
  digitalWrite(XSHT_PIN_R, LOW);
  digitalWrite(XSHT_PIN_T, LOW);
  delay(10);

  // activating LEFT
  digitalWrite(XSHT_PIN_L, HIGH);
  delay(10);
  while(!left_TOF.begin(TOF_L_ADDRESS)) {
    Serial.print(F("xx Failed to boot LEFT VL53L0X... "));
  }
  Serial.println(" ");
  Serial.println(F(" -- Booted LEFT VL53L0X successfully!"));
  delay(10);

  // activating TOP
  digitalWrite(XSHT_PIN_T, HIGH);
  delay(10);
  while(!top_TOF.begin(TOF_T_ADDRESS)) {
    Serial.println(F("xx Failed to boot OBSTACLE VL53L0X"));
  }
  Serial.println(" ");
  Serial.println(F(" -- Booted TOP VL53L0X successfully!"));
  delay(10);

  // activating RIGHT
  digitalWrite(XSHT_PIN_R, HIGH);
  delay(10);
  while(!right_TOF.begin(TOF_R_ADDRESS)) {
    Serial.println(F("xx Failed to boot RIGHT VL53L0X"));
  }
  Serial.println(" ");
  Serial.println(F(" -- Booted RIGHT VL53L0X successfully! "));
  delay(10);
  
  // start continuous ranging
  left_TOF.startRangeContinuous();
  right_TOF.startRangeContinuous();
  top_TOF.startRangeContinuous(); 

  Serial.println(" ! --- All 3 VL53L0X sensors are connected! ---"); 
}

void setup_lineIR(){
  analogSetPinAttenuation(L_IR_ANALOG_PIN, ADC_11db);
  analogSetPinAttenuation(R_IR_ANALOG_PIN, ADC_11db);
  analogSetPinAttenuation(MR_IR_ANALOG_PIN, ADC_11db);
  analogSetPinAttenuation(ML_IR_ANALOG_PIN, ADC_11db);

  //try IR receiver module
  //IrReceiver.begin(IR_DETECTOR_PIN , ENABLE_LED_FEEDBACK);
  //pinMode(IR_DETECTOR_PIN, INPUT);
  analogSetPinAttenuation(IR_DETECTOR_PIN, ADC_11db);

  Serial.println(" ! --- Line IR sensors AND receiver are set up! ---");
}


/*==============================================================================================*/
void setup() {
  Serial.begin(115200);
  Serial.println("----- Starting Edge Detection RC !!! ----------------"); 
  delay(100); // allow me to switch to serial monitor

  setup_motors();
  setup_lineIR();
  setup_obsAvoidance();// pos=90
  setup_tof(); 

  Serial.println(" ! --- Wait 4 seconds...   -----------------------"); 
  delay(4000); // let me react
}
/*==============================================================================================*/

void loop(){
  // needed for turn timing
  unsigned long now = millis();

  // handle any obstacle dodging first ========================================================
  if (objectState == OBJECT_TURNING) {
    // if its within the turning time
    if (now - objectStateStartedAt <= TURN_TIME) {
      Serial.print(F(" ^ currently turning!"));
      return;
    }
    else{
      stop(); delay(POST_TURN_STOP_TIME);
      Serial.println(" ^ Stopped turning!");
      objectState = OBJECT_MONITORING;
      objectStateStartedAt = now;
      return;
    }
  }//

  // when the IR light is detected on the left, stop until it is off
  /*
  if(IrReceiver.decode()) {
    stop(); delay(10);

    do{
      Serial.print("!!!!IR signal detected");
      IrReceiver.resume();
      delay(100);
    } while(IrReceiver.decode());
    
    delay(300);
  }*/

  int IR_light = analogRead(IR_DETECTOR_PIN);
  Serial.print("IR: "); Serial.print(IR_light);

  if(IR_light < 2000 || isLightOn){
    //isLightOn = true;
    delay(1000);
    //lightLastOnMillis = now;
    Serial.print(" | LIGHT : ");
    Serial.print(analogRead(IR_DETECTOR_PIN));
    return;
  }

  // check for obstacles by obstaining the min distance of all ===============================
  // sweep servo and ODA
  // - move servo once it is greater, and reset time
  if (now - servoPrevMillis >= POS_INTERVAL) {
    servoPrevMillis = now;

    if(pos < (90+POS_ANG) && isCW){
      pos += POS_INC;
    } else if (pos == (90+POS_ANG) && isCW){
      isCW = false;
      //pos += POS_INC;
    } else if(pos > (90-POS_ANG) && !isCW){
      pos -= POS_INC;
    } else if(pos == (90-POS_ANG) && !isCW){
      isCW = true;
      //pos -= POS_INC;
    }
    servo.write(pos);
    //Serial.print("pos: "); Serial.print(pos);
  } 
  //get the distance, regardless if the servo moved or not
  min_obs = checkObstacles();
 
  // movement logic tree ========================================================================
  if ( min_obs > OBSTACLE_DISTANCE ) {
    // if nothing blocking, move forward
    moveForward(speed_now);
    objectState = OBJECT_MONITORING;

    //check and do line following logic as well -------------------------------------------------   
    L_lineAnalog  = analogRead(L_IR_ANALOG_PIN);
    R_lineAnalog = analogRead(R_IR_ANALOG_PIN);
    MR_lineAnalog = analogRead(MR_IR_ANALOG_PIN);
    ML_lineAnalog = analogRead(ML_IR_ANALOG_PIN);

    leftOnLine = isOnLine(L_lineAnalog, LEFT_LINE_THRESHOLD);
    rightOnLine = isOnLine(R_lineAnalog, RIGHT_LINE_THRESHOLD);
    middleRightOnLine = isOnLine(MR_lineAnalog, MIDDLE_R_LINE_THRESHOLD);
    middleLeftOnLine = isOnLine(ML_lineAnalog, MIDDLE_L_LINE_THRESHOLD);

    Serial.print(" ||| L IR: "); Serial.print(L_lineAnalog);
    Serial.print(" | ML IR: "); Serial.print(ML_lineAnalog);
    Serial.print(" | MR IR: "); Serial.print(MR_lineAnalog);
    Serial.print(" | R IR: "); Serial.print(R_lineAnalog);

    Serial.print("   ||   L line on?: "); Serial.print(leftOnLine);
    Serial.print(" | ML line on?: "); Serial.print(middleLeftOnLine);
    Serial.print(" | MR line on?: "); Serial.print(middleRightOnLine);
    Serial.print(" | R line on?: "); Serial.println(rightOnLine);

    // very left
    if(leftOnLine && !middleLeftOnLine && !middleRightOnLine && !rightOnLine){//1000
      turnLeftOnSpot(SPEED_MIN+8); //delay(50);
      //Serial.println("On the line - going left");
    } 
    //little left
    else if(!leftOnLine && middleLeftOnLine && !middleRightOnLine && !rightOnLine ) {//0100
      turnLeftOnSpot(SPEED_MIN); //delay(50);
      //Serial.println("On the line -  going right");
    }
    // little right
    else if(!leftOnLine && !middleLeftOnLine && middleRightOnLine && !rightOnLine){//0010
      turnRightOnSpot(SPEED_MIN); //elay(150);
      //Serial.println("On the line - going left");
      //lineAlreadyFound = true;
    } 
    // very right
    else if(!leftOnLine && !middleLeftOnLine && !middleRightOnLine && rightOnLine) {//0001
      turnRightOnSpot(SPEED_MIN+8); //delay(150);
    }
    // sharp turns
    else if(leftOnLine && middleLeftOnLine && !middleRightOnLine && !rightOnLine ){//1100
      turnLeftOnSpot(SPEED_MAX); //delay(100);
    } 
    else if(!leftOnLine && !middleLeftOnLine && middleRightOnLine && rightOnLine){
      turnRightOnSpot(SPEED_MAX); //delay(100);
    }
    // super sharp tunrs
    else if(leftOnLine && middleLeftOnLine && middleRightOnLine ){//1100
      turnLeftOnSpot(SPEED_MAX); delay(110);
    } 
    else if(middleLeftOnLine && middleRightOnLine && rightOnLine){
      turnRightOnSpot(SPEED_MAX); delay(110);
    }
    // WEIRD sharp tunrs
    else if(leftOnLine && middleRightOnLine){//1100
      turnLeftOnSpot(SPEED_MAX); delay(150);
    } 
    else if(middleLeftOnLine && rightOnLine){
      turnRightOnSpot(SPEED_MAX); delay(150);
    }
  }// END NO OBSTACLE / LINE IF SIDE
  // obstacle detected, handle it over anything else
  else{
    stop(); delay(STOP_TIME);
    Serial.print(" ! STOP ! : ");

    // change states
    objectState = OBJECT_TURNING;
    lineAlreadyFound = false;

    //  look right and left for all turns
    float distanceR = checkRight();//1000
    //servo.write(90); delay(500); // in between to reduce s
    float distanceL = checkLeft();//1000
    servo.write(90); delay(100); // in between to reduce s

    // record the state change time now
    objectStateStartedAt = millis();//now + 2000 + 100; // now doesnt update. have to manually add 

    Serial.print("OBJECT found at: "); Serial.println(pos);
    //Serial.print(" | left: "); Serial.print(distanceL); 
    //Serial.print(" | right: "); Serial.println(distanceR); 

    if (distanceR >= distanceL && distanceR > OBSTACLE_DISTANCE*0.7)
      turnRightOnSpot(SPEED_MIN);
    else if (distanceL >= distanceR && distanceL > OBSTACLE_DISTANCE*0.7)
      turnLeftOnSpot(SPEED_MIN);
    else
      moveBackwards(SPEED_MIN+13);

  }// END MOVEMENT LOGIC TREE
  
}


/*========================================= HELPERS ========================================*/

/* ***********************************************************
 * LINE FOLLOWING FUNCTIONS
 *************************************************************/
bool isOnLine(int value, int threshold) {
  if (value >= threshold)
    return true;
  else
    return false;
}

void getLineIR(){
  L_lineAnalog = analogRead(L_IR_ANALOG_PIN);
  Serial.print("L IR analog value: ");  Serial.print(L_lineAnalog); 

  R_lineAnalog = analogRead(R_IR_ANALOG_PIN);
  Serial.print(" | R IR analog value: ");  Serial.print(R_lineAnalog); 

  MR_lineAnalog = analogRead(MR_IR_ANALOG_PIN);
  Serial.print(" | M IR analog value: ");  Serial.println(MR_lineAnalog); 
}


/* ***********************************************************
 * OBSTACLE AVOIDANCE FUNCTIONS
 *************************************************************/
float checkRight(){
  float r1 = look(0); //500
  float r2 = look(20); //500
  float distanceR = min(r1, r2);
  return distanceR;
}
float checkLeft(){
  float l1 = look(160); //500
  float l2 = look(180); //500
  float distanceL = min(l1, l2); 
  return distanceL;
}

float look(int angle){
  // turn to specified angle to look for objects
  servo.write(angle); delay(500);

  // get and return min distance found  
  return checkObstacles();
}

float checkObstacles(){
  // get all distances
  float distances[4];
  distances[0] = pingUltrasonic();
  distances[1] = float(   getToF_top() );
  distances[2] = float( getToF_right() );
  distances[3] = float(  getToF_left() );
  
  // find the smallest of them all
  float min_distance = INT_MAX;
  //Serial.print(" us | top | R | L --" );
  for(int i = 0; i < 4 ; i++){
    //Serial.print(" | ");  Serial.print(distances[i]); Serial.print(" mm"); 
    if( distances[i] <= min_distance )
      min_distance = distances[i] ;
  }
  //Serial.print(" | min is:" ); Serial.println(min_distance);
  
  return min_distance;
}

float pingUltrasonic(){
  // ping trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  float distance_mm = 0.017 * duration_us * 10.0; // x10 for cm to mm

  return distance_mm;
}
// tof distance functions for each
int getToF_top(){
  if(top_TOF.isRangeComplete()){
    return top_TOF.readRangeResult();
  } 
    
  else
    return min_obs;
}
int getToF_right(){
  if(right_TOF.isRangeComplete()) 
    return right_TOF.readRangeResult();
  else
    return min_obs;
}
int getToF_left(){
  if(left_TOF.isRangeComplete()) 
    return left_TOF.readRangeResult();
  else
    return min_obs;
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
    ledcWrite(pinA, 0);
    ledcWrite(pinB, speed);
  } else if (speed < 0) {
    ledcWrite(pinA, -speed);
    ledcWrite(pinB, 0);
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
  //Serial.println("Moving Forward");
}
void moveBackwards(int speed){
  motorA(-speed);
  motorB(-speed);
  Serial.println("Moving Backwards");
}
void turnRightOnSpot(int speed){
  // LEFT/A wheel forwards
  motorA(-speed);
  // RIGHT/B wheel backwards
  motorB(speed);
  Serial.println("Turning Right");
}
void turnLeftOnSpot(int speed){
  // LEFT/A wheel backwards
  motorA(speed);
  // RIGHT/B wheel forwards
  motorB(-speed);
  Serial.println("Turning Left");
}
void turnRightForward(int speed){
  // LEFT/A wheel forwards
  motorA(speed);
  // RIGHT/B wheel backwards
  motorB(0);
  Serial.println("Turning Right");
}
void turnLeftForward(int speed){
  // LEFT/A wheel backwards
  motorA(0);
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

void testMotorDirections(){
  moveForward(speed_now);      delay(10000);
  stop(); delay(1000);
  moveBackwards(speed_now);    delay(10000);
  stop(); delay(1000);
  turnRightOnSpot(speed_now);  delay(10000);
  stop(); delay(1000);
  turnLeftOnSpot(speed_now);   delay(10000);
  stop(); delay(1000);
  turnRightForward(speed_now); delay(10000);
  stop(); delay(1000);
  turnLeftForward(speed_now);  delay(10000);
  stop(); delay(1000);
}
