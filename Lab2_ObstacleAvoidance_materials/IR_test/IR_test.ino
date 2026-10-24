/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-infrared-obstacle-avoidance-sensor
 */

 // under a 100 analog => white
 // cant detect object more than 5cm away

#define IR_DIGITAL_PIN 34 // ESP32 pin GPIO18 connected to OUT pin of IR obstacle avoidance sensor
#define IR_ANALOG_PIN 35

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // initialize the Arduino's pin as aninput
  pinMode(IR_DIGITAL_PIN, INPUT);

  analogReadResolution(12); // ESP32 default: 0 to 4095
  analogSetPinAttenuation(IR_ANALOG_PIN, ADC_11db);
}

void loop() {
  // read the state of the the input pin:
  int IR_digital = digitalRead(IR_DIGITAL_PIN);

  if (IR_digital == LOW)
    Serial.println("The obstacle is present");
  else
    Serial.println("The obstacle is NOT present");

  int analogValue = analogRead(IR_ANALOG_PIN);

  Serial.print("IR analog value: ");
  Serial.println(analogValue);

  delay(100);
}
