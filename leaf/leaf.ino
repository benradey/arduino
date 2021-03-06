
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define LIS3DH_CS 10
#define LED_GND 8
#define LED_R 5
#define LED_Y 3
#define LED_G 6
#define RELAY_GND 0
#define RELAY_IN1 2
#define RELAY_IN2 4
#define RESET 7

#define G_MULTIPLIER 0x3fff
#define LIGHT_ON G_MULTIPLIER * 0.2
#define LIGHT_OFF G_MULTIPLIER * 0.1
#define LED_DUTY_CYCLE 10
#define LED_DC_YELLOW_MOD 20

Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

boolean lit = false;

#define AVG_LENGTH 20
double avg[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int avgIndex = 0;

void setup() {
  pinMode(RESET, INPUT);

  Serial.begin(9600);
  Serial.println("LIS3DH BEN test!");

  pinMode(LED_GND, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(RELAY_GND, OUTPUT);
  pinMode(RELAY_IN1, OUTPUT);
  pinMode(RELAY_IN2, OUTPUT);

  digitalWrite(LED_GND, LOW);
  digitalWrite(RELAY_GND, LOW);
  digitalWrite(RELAY_IN1, HIGH);
  digitalWrite(RELAY_IN2, HIGH);
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    delay(10000);
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, LOW);
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  analogWrite(LED_G, LED_DUTY_CYCLE);
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

  int xg = lis.x;
  xg = -xg;

  if (xg > LIGHT_ON) {
    analogWrite(LED_Y, LED_DUTY_CYCLE + LED_DC_YELLOW_MOD);
  } else if (xg < LIGHT_OFF) {
    analogWrite(LED_Y, 0);
  }

  avg[avgIndex] = xg;
  avgIndex++;
  if (avgIndex >= AVG_LENGTH) {
    avgIndex = 0;
  }
  
  long avgg = 0;
  for (int i = AVG_LENGTH - 1 ; i > 0 ; i--) {
    avgg += avg[i];
  }

  avgg = avgg / AVG_LENGTH;

  if (lit && avgg < LIGHT_OFF) {
    Serial.println("Lights off");
    analogWrite(LED_R, 0);
    digitalWrite(RELAY_IN1, HIGH);
    lit = false;
  }
  if (!lit && avgg > LIGHT_ON) {
    Serial.println("Lights on");
    analogWrite(LED_R, LED_DUTY_CYCLE);
    digitalWrite(RELAY_IN1, LOW);
    lit = true;
  }
 
  delay(20);
}
