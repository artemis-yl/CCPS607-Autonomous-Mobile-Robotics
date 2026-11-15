/* ***********************************************************
 * LIBRARIES
 *************************************************************/
#include <Wire.h> // for vl530l0x
#include "Adafruit_VL53L0X.h" // for vl530l0x
#include <ESP32Servo.h> // for the servo

/* ***********************************************************
 * ESP32 PINS USED
 *************************************************************/
/* vl530l0x xshut pins - 3.3V */
#define XSHT_PIN_L  23
#define XSHT_PIN_R  15
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
#define R_IR_ANALOG_PIN 33
#define M_IR_ANALOG_PIN 32

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
const int SPEED_MAX = 230; // untested
const int SPEED_MIN = 187; // 180 isnt enough for A/L. 185 has some issues
int speed_now = SPEED_MIN; // 
int turn_speed = SPEED_MIN ;  

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
const int OBSTACLE_DISTANCE = 300; // apply both for TOF and ultrasonic
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
const int POS_INC = 30; // the dif between each stopped position
const int POS_ANG = 30; // +/- from 90deg
const int POS_INTERVAL = 100; //20ms is the fastest the servo can go
// 10, 40, 40 was previous values
unsigned long servoPrevMillis = 0; // used to time sweep increments

/* ***********************************************************
 * LINE FOLLOWING VARIABLES
 *************************************************************/
const int LEFT_LINE_THRESHOLD   = 800; // high 200 -> 1100s
const int RIGHT_LINE_THRESHOLD  = 600; // high 200 -> 800s
const int MIDDLE_LINE_THRESHOLD = 3000; // low 300s -> 3700s


int L_lineAnalog, R_lineAnalog, M_lineAnalog;
bool lineAlreadyFound = false;
bool leftOnLine = false;
bool rightOnLine = false;
bool middleOnLine = false;

unsigned long middleLinePrevMillis = 0;
const int LINE_INTERVAL = 1000;

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
  //pinMode(L_IR_DIGITAL_PIN, INPUT);
  //analogReadResolution(12); // ESP32 default: 0 to 4095
  analogSetPinAttenuation(L_IR_ANALOG_PIN, ADC_11db);

  //pinMode(R_IR_DIGITAL_PIN, INPUT);
  //analogReadResolution(12); // ESP32 default: 0 to 4095
  analogSetPinAttenuation(R_IR_ANALOG_PIN, ADC_11db);

  analogSetPinAttenuation(M_IR_ANALOG_PIN, ADC_11db);

  Serial.println(" ! --- Line IR sensors are set up! ---");
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

void loop() {
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
  }
  //get the distance, regardless if the servo moved or not
  min_obs = checkObstacles();

  //getLineIR(); // just see what happens for now

  // movement logic tree ========================================================================
  if ( min_obs > OBSTACLE_DISTANCE ) {
    // if nothing blocking, move forward
    moveForward(speed_now);
    objectState = OBJECT_MONITORING;

    //check and do line following logic as well
/*
    // check if either see a line
    getLineIR(); // just see what happens for now
    int leftValue  = analogRead(L_IR_ANALOG_PIN);
    int rightValue = analogRead(R_IR_ANALOG_PIN);
    int middleValue = analogRead(M_IR_ANALOG_PIN);

    leftOnLine = isOnLine(leftValue, LEFT_LINE_THRESHOLD);
    rightOnLine = isOnLine(rightValue, RIGHT_LINE_THRESHOLD);
    middleOnLine = isOnLine(middleValue, MIDDLE_LINE_THRESHOLD);

    // already on the line
    if(lineAlreadyFound){
      // either sensor sees, move forward
      if(!leftOnLine && !rightOnLine || middleOnLine) moveForward(speed_now);
      // if only left sensor detects, needs to turn left
      else if(leftOnLine && !rightOnLine) turnLeftForward(speed_now);
      // if right sensor detects, needs to turn right
      else if(!leftOnLine && rightOnLine) turnRightForward(speed_now);

      //if ran out of line or lost it
      if(middleOnLine)
        middleLinePrevMillis = now;
      else{
        // if middle havent seen it in a while
        if(now - middleLinePrevMillis > LINE_INTERVAL){
          lineAlreadyFound = false;
        }
      }
    }
    else{
      // just found the line
      // - either way, move foward until middle sensor finds it
      // - then turn until a sensor finds and unfinds
      // left sensor 1st, so line is down-left to up-right
      if(leftOnLine && !rightOnLine){
        Serial.print(">| | Left Sees Line - ");

        //move forward until middle sees line
        while(!middleOnLine ) moveForward(SPEED_MIN);        
        Serial.print(" | Middle on Line. Now turning RIGHT to refind! ");

        // turn right until R_IR sees...
        while(!rightOnLine) turnRightOnSpot(speed_now);       
        Serial.print(" | RIGHT IR sees! "); 

        // ...and unsees
        while(rightOnLine) turnRightOnSpot(speed_now);       
        Serial.print(" | RIGHT no longer sees! Should be between L and L IRs! "); 
      }
      else{//((leftOnLine && !rightOnLine) || (leftOnLine && rightOnLine))
        Serial.print(">| | Right Sees Line - ");

        //move forward until middle sees line
        while(!middleOnLine ) moveForward(SPEED_MIN);        
        Serial.print(" | Middle on Line. Now turning LEFT to refind! ");

        // turn right until R_IR sees...
        while(!rightOnLine) turnLeftOnSpot(speed_now);       
        Serial.print(" | RIGHT IR sees! "); 

        // ...and unsees
        while(rightOnLine) turnLeftOnSpot(speed_now);       
        Serial.print(" | RIGHT no longer sees! Should be between L and L IRs! "); 
      }
      lineAlreadyFound = true;

    }
*/
    
    // when the IR light is detected on the left, stop until it is off
    //while(IR_detected){
    //  stop(); delay(10);
   // }
    
  }
  // obstacle detected, handle it over anything else
  else{
    stop(); delay(STOP_TIME);
    Serial.print(" ! STOP ! : ");

    // change states
    objectState = OBJECT_TURNING;
    lineAlreadyFound = false;

    //  look right and left for all turns
    float distanceR = checkRight();//1400
    //servo.write(90); delay(500); // in between to reduce s
    float distanceL = checkLeft();//1400
    servo.write(90); delay(100); // in between to reduce s

    // record the state change time now
    objectStateStartedAt = now + 2800 + 100; // now doesnt update. have to manually add 

    Serial.print("OBJECT found at: "); Serial.println(pos);
    //Serial.print(" | left: "); Serial.print(distanceL); 
    //Serial.print(" | right: "); Serial.println(distanceR); 

    if (distanceR >= distanceL && distanceR > OBSTACLE_DISTANCE*0.8)
      turnRightOnSpot(turn_speed);
    else if (distanceL >= distanceR && distanceL > OBSTACLE_DISTANCE*0.8)
      turnLeftOnSpot(turn_speed);
    else
      moveBackwards(speed_now);

    
  }
  

}
/*==============================================================================================*/


/* ***********************************************************
 * LINE FOLLOWING FUNCTIONS
 *************************************************************/
bool isOnLine(int value, int threshold) {
  return true ? value > threshold : value < threshold;
}

void getLineIR(){

  L_lineAnalog = analogRead(L_IR_ANALOG_PIN);
  Serial.print("L IR analog value: ");  Serial.print(L_lineAnalog); 

  R_lineAnalog = analogRead(R_IR_ANALOG_PIN);
  Serial.print(" | R IR analog value: ");  Serial.print(R_lineAnalog); 

  M_lineAnalog = analogRead(M_IR_ANALOG_PIN);
  Serial.print(" | M IR analog value: ");  Serial.println(M_lineAnalog); 
}



/* ***********************************************************
 * OBSTACLE AVOIDANCE FUNCTIONS
 *************************************************************/
float checkRight(){
  float r1 = look(0); //700
  float r2 = look(20); //700
  float distanceR = min(r1, r2);
  return distanceR;
}
float checkLeft(){
  float l1 = look(160); //700
  float l2 = look(180); //700
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
  Serial.print(" us | top | R | L --" );
  for(int i = 0; i < 4 ; i++){
    Serial.print(" | ");  Serial.print(distances[i]); Serial.print(" mm"); 
    if( distances[i] <= min_distance )
      min_distance = distances[i] ;
  }
  Serial.print(" | min is:" ); Serial.println(min_distance);
  
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
  if(top_TOF.isRangeComplete()) 
    return top_TOF.readRangeResult();
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
void turnRightOnSpot(int speed){
  // LEFT/A wheel forwards
  motorA(speed);
  // RIGHT/B wheel backwards
  motorB(-speed);
  Serial.println("Turning Right");
}
void turnLeftOnSpot(int speed){
  // LEFT/A wheel backwards
  motorA(-speed);
  // RIGHT/B wheel forwards
  motorB(speed);
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
