/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-infrared-obstacle-avoidance-sensor
 */

 // under a 100 analog => white
 // cant detect object more than 5cm away

#define L_IR_DIGITAL_PIN 34 // ESP32 pin GPIO18 connected to OUT pin of IR obstacle avoidance sensor
#define L_IR_ANALOG_PIN 35
#define R_IR_DIGITAL_PIN 32 // ESP32 pin GPIO18 connected to OUT pin of IR obstacle avoidance sensor
#define R_IR_ANALOG_PIN 33

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // initialize the Arduino's pin as aninput
  pinMode(L_IR_DIGITAL_PIN, INPUT);

  analogReadResolution(12); // ESP32 default: 0 to 4095
  analogSetPinAttenuation(L_IR_ANALOG_PIN, ADC_11db);
/*
  // initialize the Arduino's pin as aninput
  pinMode(R_IR_DIGITAL_PIN, INPUT);

  analogReadResolution(12); // ESP32 default: 0 to 4095
  analogSetPinAttenuation(R_IR_ANALOG_PIN, ADC_11db);
  */
}

void loop() {
  // read the state of the the input pin:
  int L_IR_digital = digitalRead(L_IR_DIGITAL_PIN);
  if (L_IR_digital == LOW)
    Serial.print("The L obstacle is present");
  else
    Serial.print("The L obstacle is NOT present");
  int analogValue = analogRead(L_IR_ANALOG_PIN);
  Serial.print(" | L IR analog value: ");
  Serial.println(analogValue);
/*
  int R_IR_digital = digitalRead(R_IR_DIGITAL_PIN);
  if (R_IR_digital == LOW)
    Serial.print(" || The R obstacle is present");
  else
    Serial.print("The R obstacle is NOT present");

  analogValue = analogRead(R_IR_ANALOG_PIN);

  Serial.print(" | R IR analog value: ");
  Serial.println(analogValue);
*/
  delay(100);
}
