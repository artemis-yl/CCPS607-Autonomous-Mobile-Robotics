/* ***********************************************************
 * VL53L0X / EDGE DETECTION VARIABLES
 *************************************************************/
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define SDA_R 21 // default
#define SCL_R 22 
#define SDA_L 19 // extra to set, not gonna deal with xshut nor i2c multiplexer
#define SCL_L 23
// #define VL53L0X_I2C_ADDR 0x29

TwoWire I2C_R = TwoWire(0);
TwoWire I2C_L = TwoWire(1);

Adafruit_VL53L0X right_TOF;// = Adafruit_VL53L0X();
Adafruit_VL53L0X left_TOF; // = Adafruit_VL53L0X();

const int EDGE_DISTANCE = 60; // sensor are 40-50mm away


/* ***********************************************************
 * MOTOR CONTROL VARIABLES
 *************************************************************/
// Motor A connections
#define EN_A         14  // EN_A pin
#define MOTOR_A_IN_1 27  // IN1 pin
#define MOTOR_A_IN_2 26  // IN2 pin

// Motor B connections
#define MOTOR_B_IN_3 25  // IN3 pin
#define MOTOR_B_IN_4 33  // IN4 pin
#define EN_B         32  // EN_B pin

// PWM properties
const int FREQ = 1000;        // PWM FREQuency
const int PWM_CHANNEL_A = 0; // PWM channel for motor A, 0-15
const int PWM_CHANNEL_B = 1; // PWM channel for motor B, 0-15
const int RESOLUTION = 8;    // 1-16bits. 8bit -> 0-255 duty cycle
const int MOTOR_SPEED = 200; // duty cycle = 0 - 255. @ 9v this is good
const int MIN_SPEED = 130;   // testing shows both can go from min 130


void setup_tof(){
  // initialize 2 I2C buses, 1 default 1 custom on 32, 33
  I2C_R.begin(SDA_R, SCL_R, 100000); // default
  I2C_L.begin(SDA_L, SCL_L, 100000);

  if (!right_
TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_R
) ) {
    Serial.println("Could not find a valid VL53L0X_1/r sensor, check wiring!");
    while (1);
  }
  
  if (!left_TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_L) ) {
    Serial.println("Could not find a valid VL53L0X_2/l sensor, check wiring!");
    while (1);
  }

  // start continuous ranging
  right_
TOF.startRangeContinuous();
  left_TOF.startRangeContinuous();
}

void setup_motors(){
  // Set all the motor control pins to outputs
  pinMode(MOTOR_A_IN_1, OUTPUT);
  pinMode(MOTOR_A_IN_2, OUTPUT);
  pinMode(MOTOR_B_IN_3, OUTPUT);
  pinMode(MOTOR_B_IN_4, OUTPUT);
  
  // Configure PWM for motor speed control using new API
  ledcAttachChannel(EN_A, FREQ, RESOLUTION, PWM_CHANNEL_A);
  ledcAttachChannel(EN_B, FREQ, RESOLUTION, PWM_CHANNEL_B);
  
  // Turn off motors - Initial state
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, LOW);
  
  // Set initial PWM duty cycle to 0 (motors off)
  ledcWrite(EN_A, 0);
  ledcWrite(EN_B, 0);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Edge Detection RC..."); 

  /************************************************************
   * VL53L0X / EDGE DETECTION SETUP
   *************************************************************/
  setup_tof();
  Serial.println("Both VL53L0X sensors are connected!"); 

  /* ***********************************************************
   * MOTOR CONTROL SETUP
   *************************************************************/
  setup_motors();
  Serial.println("Both motors are set up!");
}


void loop() {
  //testMotorMovement();
  //testTOFsensors();

  // first make sure readings availible
  if (left_TOF.isRangeComplete() && right_TOF.isRangeComplete()) {
    int right_d = right_TOF.readRange();
    int left_d = left_TOF.readRange();

    /* THE ALG
    // aka no edge detected
    if ( if both left and right < XX mm )
      - forward
    // edge detected - both turn types, stop first then back a wee bit
    else
      - stop
      - back a lil (very lil! edge case is rover place right at a corner diagonally)

      // turn right either way
      // aka edge right in front or at left side  => turn right
      if both left and right > XX mm || if left >= XX mm and right < XXmm :  
        - turn right
      //  aka edge at right side => turn left
      else if left < XX mm and right >= XXmm 
        - turn  left
    */

    // no edge detected => on table safe
    if(left_d <= EDGE_DISTANCE && right_d <= EDGE_DISTANCE){
      moveForward();
    }
    // edge detected
    else{
      stop(); delay(1000); //stop a bit
      moveBackwards(); delay(500) // move backwards very lil
/*
      // edge ahead or at left egde
      if( (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE)) || (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE)) ){
        turnRight(); delay(5000); // turn away from edge
      }
      else if(left_d <= EDGE_DISTANCE && right_d > EDGE_DISTANCE){
        turnLeft(); delay(5000);
      }
*/
    }// end ELSE
}


/* ***********************************************************
 * VL53L0X / EDGE DETECTION TEST FUNCTIONS
 *************************************************************/

void testTOFsensors(){
  if (right_
TOF.isRangeComplete() && left_TOF.isRangeComplete() ) {
    Serial.print("RIGHT Distance in mm: ");
    Serial.print(right_
  TOF.readRange());

    Serial.print(" | LEFT Distance in mm: ");
    Serial.println(left_TOF.readRange());
  }
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

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Moving Forward");
}
void moveBackwards(){
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Moving Backwards");

}
void turnRight(){
  // right/A wheel BACKWARDS
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  // left/B wheel FORWARDS
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Turning Right");
}
void turnLeft(){
  // right/A wheel forward
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  // left/B wheel backwards
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Turning Left");
}
// Turn off motors
void stop() {
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, 0);
  ledcWrite(EN_B, 0);

  Serial.println("Motor stopped");
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

