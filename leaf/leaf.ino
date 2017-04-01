
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define LIS3DH_CS 10
#define LIGHT_ON 0.2
#define LIGHT_OFF 0.1

Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

boolean lit = false;

void setup() {

  Serial.begin(9600);
  Serial.println("LIS3DH BEN test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

}

void loop() {
  
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  //Serial.print("X:  "); Serial.print(lis.x); Serial.print(" / "); Serial.print(0x7fff);
  //Serial.print("  \tY:  "); Serial.print(lis.y); 
  //Serial.print("  \tZ:  "); Serial.print(lis.z); 

  

  /* Or....get a new sensor event, normalized */ 
  //sensors_event_t event; 
  //lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  //Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  //Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  //Serial.println(" m/s^2 ");

  //Serial.print(lis.x / (double) 0x3fff);

  double xg = lis.x / (double) 0x3fff;

  if (lit && xg < LIGHT_OFF) {
    Serial.println("Lights off");
    lit = false;
  }
  if (!lit && xg > LIGHT_ON) {
    Serial.println("Lights on");
    lit = true;
  }
  
 
  delay(20); 

}
