/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-ultrasonic-sensor
 */
#include <ESP32Servo.h>
#define SERVO_PIN 13

#define TRIG_PIN 15// ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 2  // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

Servo servo;
int pos = 0;

float duration_us, distance_cm;

bool isCW = true;

void setup() {
  // begin serial port
  Serial.begin (115200);

  delay(5000);

  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);


  servo.attach(SERVO_PIN);
  servo.write(pos);
  delay(5000);
}

void loop() {
  
  
  if(pos < 180 && isCW){
    pos += 10;
    //Serial.println("Servo turns CW");
  }
  else if (pos == 180 && isCW){
    isCW = false;
  }
  else if(pos > 0 && !isCW){
    pos -= 10;
    //Serial.println("Servo turns CCW");
  }
  else if(pos == 0 && !isCW){
    isCW = true;
  }

  Serial.print("pos is: "); Serial.print(pos);
    
  servo.write(pos);

  //delay(); // waits 15ms to reach the position
  

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH, 20000);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print(" | distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(10);
}
