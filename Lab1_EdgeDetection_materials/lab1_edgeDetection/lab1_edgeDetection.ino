/* ***********************************************************
 * VL53L0X / EDGE DETECTION VARIABLES
 *************************************************************/
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define SDA_R 19 // extra to set, not gonna deal with xshut nor i2c multiplexer
#define SCL_R 23 
#define SDA_L 21 // default
#define SCL_L 22
// #define VL53L0X_I2C_ADDR 0x29

TwoWire I2C_R = TwoWire(0);
TwoWire I2C_L = TwoWire(1);

Adafruit_VL53L0X right_TOF;// = Adafruit_VL53L0X();
Adafruit_VL53L0X left_TOF; // 
Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_configR = Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY;
Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_configL = Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY;

const int EDGE_DISTANCE = 65; // sensor are 40-50mm away

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
const int MOTOR_SPEED_MAX = 190; // duty cycle = 0 - 255. @ 5v this is good
const int MOTOR_SPEED_MIN = 160;   // testing shows both can go from min 130 @ 9V, but 5V needs more


void setup_tof(){
  // initialize 2 I2C buses, 1 default 1 custom on 32, 33
  I2C_R.begin(SDA_R, SCL_R, 100000); // default
  I2C_L.begin(SDA_L, SCL_L, 100000);

  if (!right_TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_R, sensor_configR) ) {
    Serial.println("Could not find a valid VL53L0X_1/r sensor, check wiring!");
    while (1);
  }
  if (!left_TOF.begin(VL53L0X_I2C_ADDR, false, &I2C_L, sensor_configL) ) {
    Serial.println("Could not find a valid VL53L0X_2/l sensor, check wiring!");
    while (1);
  }

  // start continuous ranging
  right_TOF.startRangeContinuous();
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
  delay(1000); // allow me to switch to serial monitor

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

  testEdge();
}

void testEdge(){
  // check that both TOFs are BOTH ready
  if (!readTOFs()) {
    return;
  }
  uint16_t left_d = left_TOF.readRangeResult();
  uint16_t right_d = right_TOF.readRangeResult();

  Serial.print("Right: ");
  Serial.print(right_d);
  Serial.print(" mm, Left: ");
  Serial.print(left_d);
  Serial.print(" mm ==> ");

  if (left_d <= EDGE_DISTANCE && right_d <= EDGE_DISTANCE) {
    moveForward();
  } 
  else {
    stop();          delay(500);
    moveBackwards(); delay(500);
    stop();          delay(500);

    // edge ahead 
    if (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE){ 
      Serial.print("Egde Ahead! -> ");
      turnRight(); delay(5000); // turn away from edge
    }
    // edge at left
    else if (left_d > EDGE_DISTANCE && right_d <= EDGE_DISTANCE){
      Serial.print("Egde LEFT! -> ");
      turnRight(); delay(5000); // turn away from edge
    }
    //edge at right
    else if(left_d <= EDGE_DISTANCE && right_d > EDGE_DISTANCE){
      Serial.print("Egde RIGHT! -> ");
      turnLeft(); delay(5000);
    }

    
  }
}


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
  if (!left_TOF.isRangeComplete() ||
      !right_TOF.isRangeComplete()) {
    return false;
  }
  return true;
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

  ledcWrite(EN_A, MOTOR_SPEED_MAX);
  ledcWrite(EN_B, MOTOR_SPEED_MAX);

  Serial.println("Moving Forward");
}
void moveBackwards(){
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED_MIN);
  ledcWrite(EN_B, MOTOR_SPEED_MIN);

  Serial.println("Moving Backwards");

}
void turnRight(){
  // right/A wheel BACKWARDS
  digitalWrite(MOTOR_A_IN_1, HIGH);
  digitalWrite(MOTOR_A_IN_2, LOW);
  // left/B wheel FORWARDS
  digitalWrite(MOTOR_B_IN_3, LOW);
  digitalWrite(MOTOR_B_IN_4, HIGH);

  ledcWrite(EN_A, MOTOR_SPEED_MAX);
  ledcWrite(EN_B, MOTOR_SPEED_MAX);

  Serial.println("Turning Right");
}
void turnLeft(){
  // right/A wheel forward
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  // left/B wheel backwards
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A, MOTOR_SPEED_MAX);
  ledcWrite(EN_B, MOTOR_SPEED_MAX);

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
