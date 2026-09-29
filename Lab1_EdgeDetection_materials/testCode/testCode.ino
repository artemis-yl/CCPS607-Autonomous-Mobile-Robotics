// NEED TO TEST TURN DUTYCYCLES


// Motor A - LEFT WHEEL
int enA = 14; 
int motorAPin1 = 27; //A+
int motorAPin2 = 26; //A-

// Motor B - RIGHT WHEEL
int motorBPin3 = 25;  //B+
int motorBPin4 = 33;  //B-
int enB = 32; 

// Setting PWM properties
const int freq = 1000;
const int pwmChannelA = 0; // 0-15
const int pwmChannelB = 1; // 0-15
const int resolution = 8; //1-16bits. 8bit -> 0-255 duty cycle
int dutyCycleDefault = 200; //0 - 255

void setup() {
  // sets the pins as outputs:
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  //pinMode(enableAPin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  //pinMode(enableBPin, OUTPUT);
  
  // configure LEDC PWM
  ledcAttachChannel(enA, freq, resolution, pwmChannelA);
  ledcAttachChannel(enB, freq, resolution, pwmChannelB);

  // Set initial PWM duty cycle to 0 (motors off)
  ledcWrite(enA, 0);
  ledcWrite(enB, 0);

  Serial.begin(115200);

  // testing
  Serial.println("Testing DC Motor...");
}

void loop() {
  //set speed 
  //ledcWrite(enA, speed);
  //ledcWrite(enB, speed);

  // test min duty
  testMin();
}

// to find lowest dutyCycle
void testMin(){
  for (int duty = 0; duty <= 255; duty += 5) {
    digitalWrite(ENA, duty);
    Serial.println(duty);
    delay(500);                 // long enough to see it start
  }
}

void moveForward(){
  ledcWrite(enA, dutyCycleDefault);
  ledcWrite(enB, dutyCycleDefault);

  Serial.println("Moving Forward");
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin3, HIGH);
  digitalWrite(motorBPin4, LOW);
}

void moveBackwards(){
  ledcWrite(enA, dutyCycleDefault);
  ledcWrite(enB, dutyCycleDefault);

  Serial.println("Moving Backwards");
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin3, LOW);
  digitalWrite(motorBPin4, HIGH);
}

// https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot

// both turns are on the spot i.e. rotate on its center
void turnRight(){
  ledcWrite(enA, dutyCycleDefault-50);
  ledcWrite(enB, dutyCycleDefault-50);

  Serial.println("Turning Right");
  // left wheel forward
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  // right wheel backwards
  digitalWrite(motorBPin3, LOW);
  digitalWrite(motorBPin4, HIGH);
}

void turnLeft(int speed){
  ledcWrite(enA, dutyCycleDefault-50);
  ledcWrite(enB, dutyCycleDefault-50);

  Serial.println("Turning Right");
  // left wheel BACKWARDS
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  // right wheel FORWARDS
  digitalWrite(motorBPin3, HIGH);
  digitalWrite(motorBPin4, LOW);
}

// Turn off motors
void stop() {
  Serial.println("Motor stopped");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}







// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors (reverse direction)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    ledcWrite(enA, i);
    ledcWrite(enB, i);
    delay(20);
  }
  
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    ledcWrite(enA, i);
    ledcWrite(enB, i);
    delay(20);
  }
  
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
