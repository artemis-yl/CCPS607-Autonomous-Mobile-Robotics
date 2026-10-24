/* ***********************************************************
 * ? / OBSTACLE AVOIDANCE VARIABLES
 *************************************************************/
bool isObstacle = false;
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
Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_configR = Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED;
Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_configL = Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED;
// VL53L0X_SENSE_HIGH_ACCURACY   VL53L0X_SENSE_LONG_RANGE   VL53L0X_SENSE_DEFAULT 

bool isEdge = false;

const int MAX_EDGE_DISTANCE = 75; // sensor are 40-50mm away
const unsigned long EDGE_TURN_TIME = 500;
unsigned long last_edge_time = 0; //max unsign long 4,294,967,295
 
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
const int FREQ = 30000;        // PWM FREQuency
const int PWM_CHANNEL_A = 0; // PWM channel for motor A, 0-15
const int PWM_CHANNEL_B = 1; // PWM channel for motor B, 0-15
const int RESOLUTION = 8;    // 1-16bits. 8bit -> 0-255 duty cycle
const int MOTOR_SPEED_MAX = 205; // duty cycle = 0 - 255. @ 5v this is good
const int RIGHT_PLUS = MOTOR_SPEED_MAX + 5; // testing shows both can go from min 130 @ 9V, but 5V needs more
const int MOTOR_SPEED_TURN = MOTOR_SPEED_MAX - 5;   

/* ***********************************************************
 * GENERIC TIME VARIABLES
 *************************************************************/
const int STOP_TIME = 500;
bool alreadyStopped = false;


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

/* 
  when forward?
  - no edge (tofs >= edge_safe )
  - no obstacle (OB >= obs_safe )
  - left line_IR + right line_IR do not detect 
  
  when stop?
  - edge detected -> check which tOF specifically
  - obstacle detected -> check left and right of car
  
  when backwards?
  - obstacle too close ( < 5 cm )
  
  when right?
  - if edge at left OR front
  - if obstacle at left OR front
  - if:
    - WAS_ON_LINE == true
      - if (right line_IR detects line AND NOT left detects line) 
    - WAS_ON_LINE == false
      - if left line_IR detects line AND NOT right detects line
      - if left and right line_IR both detects line 
  
  when left?
  - if edge at right
  - if obstacle at right
  - if:
    - WAS_ON_LINE == true
      - if (left line_IR detects line AND NOT right detects line) 
    - WAS_ON_LINE == false
      - if right line_IR detects line AND NOT left detects line
*/
void loop() {
  // for turn timing
  unsigned long now_milli = millis();
  
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

  // edge logic
  // no edge ahead
  if (left_d <= MAX_EDGE_DISTANCE && right_d <= MAX_EDGE_DISTANCE)
    isEdge = false;
  //edge detected
  else{
    if (!isEdge) // edge detected 1st time
      last_edge_time = now_milli; //millis() + STOP_TIME;
    isEdge = true;
  }
  // should it be turning away from an edge right now? yes, if less than turn time passed
  bool isEdgeTurnPeriod = (now_milli - last_edge_time < EDGE_TURN_TIME) ? true : false;
  // since 1000 delay at setup, always at least (1000-0=1000) at beginning with no edge

  // obstacle logic

  

  
  // move forward IF no edge and no longer in edge turning AND no obstacle 
  if (!isEdge && !isEdgeTurnPeriod) {// && noObstacle
    moveForward();
    alreadyStopped = false;
  } 
  //stop
  // - if edge and havent stopped already
  // - if edge and not turning and stopped already i.e. need to stop again
  else if (isEdge && (!alreadyStopped || !isEdgeTurnPeriod && alreadyStopped ){ // || noObstacle == false
    //noObstacle = false;
    stop();  delay(STOP_TIME);
    alreadyStopped = true;
  }

  // turn right
  // - edge was detected in front or at left, turning time
  if ((isEdgeTurnPeriod && left_d > MAX_EDGE_DISTANCE) ){
    turnRight();
    if(right_d > MAX_EDGE_DISTANCE)
      Serial.print("Edge Ahead! -> ");
    else if(right_d <= MAX_EDGE_DISTANCE)
      Serial.print("Edge LEFT! -> ");
  }
  // turn left
  // - edge was detected in right, turning time
  else if ((isEdgeTurnPeriod && left_d <= MAX_EDGE_DISTANCE && right_d > MAX_EDGE_DISTANCE) ){
    turnLeft();
    Serial.print("Edge RIGHT! -> ");
  }

/*
  // edge ahead 
  if (left_d > EDGE_DISTANCE && right_d > EDGE_DISTANCE){ 
    Serial.print("Egde Ahead! -> ");
    //stop();          delay(300);
    turnRight(); //delay(TURN_TIME); // turn away from edge
  }
  // edge at left
  else if (left_d > EDGE_DISTANCE && right_d <= EDGE_DISTANCE){
    Serial.print("Egde LEFT! -> ");
    turnRight(); //delay(TURN_TIME); // turn away from edge
  }
  //edge at right
  else if(left_d <= EDGE_DISTANCE && right_d > EDGE_DISTANCE){
    Serial.print("Egde RIGHT! -> ");
    turnLeft(); //delay(TURN_TIME);
  }
*/
  

}// END LOOP

/* ***********************************************************
 * VL53L0X / EDGE DETECTION FUNCTIONS
 *************************************************************/
/* There are 2 sensors to check
 * This functions handles checking that both TOFs are ready at the same time
 */void setup_tof(){
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

  ledcWrite(EN_A_R, RIGHT_PLUS);
  ledcWrite(EN_B_L, MOTOR_SPEED_MAX);

  Serial.println("Turning Right");
}
void turnLeft(){
  // right/A wheel forward
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, HIGH);
  // left/B wheel backwards
  digitalWrite(MOTOR_B_IN_3, HIGH);
  digitalWrite(MOTOR_B_IN_4, LOW);

  ledcWrite(EN_A_R, RIGHT_PLUS);
  ledcWrite(EN_B_L, MOTOR_SPEED_MAX);

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
