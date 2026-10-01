
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
const int FREQ = 30000;        // PWM FREQuency
const int PWM_CHANNEL_A = 0; // PWM channel for motor A, 0-15
const int PWM_CHANNEL_B = 1; // PWM channel for motor B, 0-15
const int RESOLUTION = 8;    // 1-16bits. 8bit -> 0-255 duty cycle
const int MOTOR_SPEED = 200; // duty cycle = 0 - 255. @ 9v this is good
const int MIN_SPEED = 130;   // testing shows both can go from min 130



void setup() {
  Serial.begin(115200);


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
  testMotorMovement();
}





/* ***********************************************************
  * MOTOR CONTROL - MOVEMENT FUNCTIONS
  *************************************************************/
// NOTE: 
// - MotorA is right -> clockwise=forward, MotorB is left -> CounterCW=forward
//   => flip wires ( or set inverse IN values )
// - both turns are on the spot i.e. rotate on its center
// - both motors mounted 'backwards'

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

