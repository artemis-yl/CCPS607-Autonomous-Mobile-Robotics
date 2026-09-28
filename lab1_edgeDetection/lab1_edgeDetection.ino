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

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

Adafruit_VL53L0X right_edge_TOF;// = Adafruit_VL53L0X();
Adafruit_VL53L0X left_edge_TOF; // = Adafruit_VL53L0X();


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
const int MOTOR_SPEED = 150; // duty cycle = 0 - 255. @ 9v this is good
const int MIN_SPEED = 130;   // testing shows both can go from min 130



void setup() {
  Serial.begin(115200);
  /* ***********************************************************
   * VL53L0X / EDGE DETECTION SETUP
   *************************************************************/
  // initialize 2 I2C buses, 1 default 1 custom on 32, 33
  I2C_1.begin(SDA_R, SCL_R, 100000); // default
  I2C_2.begin(SDA_L, SCL_L, 100000);

  if (!right_edge_TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_1) ) {
    Serial.println("Could not find a valid VL53L0X_1/r sensor, check wiring!");
    while (1);
  }
  
  if (!left_edge_TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_2) ) {
    Serial.println("Could not find a valid VL53L0X_2/l sensor, check wiring!");
    while (1);
  }

  // start continuous ranging
  right_edge_TOF.startRangeContinuous();
  left_edge_TOF.startRangeContinuous();
  Serial.println("Both VL53L0X sensors are connected!");

  /* ***********************************************************
   * MOTOR CONTROL SETUP
   *************************************************************/
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

  Serial.println("Both motors are set up!");
}


void loop() {
  //testMotorMovement();

  testTOFsensors();
}


/* ***********************************************************
 * VL53L0X / EDGE DETECTION SETUP
 *************************************************************/


void edgeDetection(){
  // first make sure readings availible
  if (right_edge_TOF.isRangeComplete() && left_edge_TOF.isRangeComplete() ) {
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
    
    if( right_edge_TOF.readRange() && left_edge_TOF.readRange() ){

    }
  }
}


void testTOFsensors(){
  if (right_edge_TOF.isRangeComplete() && left_edge_TOF.isRangeComplete() ) {
    Serial.print("RIGHT Distance in mm: ");
    Serial.print(right_edge_TOF.readRange());

    Serial.print(" | LEFT Distance in mm: ");
    Serial.println(left_edge_TOF.readRange());
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
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Moving Forward");
}
void moveBackwards(){
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Moving Backwards");

}
void turnRight(){
  // right/A wheel BACKWARDS
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  // left/B wheel FORWARDS
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED);
  ledcWrite(EN_B, MOTOR_SPEED);

  Serial.println("Turning Right");
}
void turnLeft(){
  // right/A wheel forward
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  // left/B wheel backwards
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

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
  delay(7000);
  moveBackwards();
  delay(7000);
  turnRight();
  delay(10000);
  turnLeft();
  delay(10000);
  stop();
  delay(7000);
}


