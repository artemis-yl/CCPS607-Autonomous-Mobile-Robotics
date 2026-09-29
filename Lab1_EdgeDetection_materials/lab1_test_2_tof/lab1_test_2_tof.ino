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

