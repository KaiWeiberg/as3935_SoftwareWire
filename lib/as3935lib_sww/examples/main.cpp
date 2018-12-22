#include <Arduino.h>
#include "I2C.h"
#include <AS3935.h>


#define IRQ_PIN 2
#define CS_PIN 9

volatile bool detected = false;

void alert() {
  detected = true;
}

void printDistance() {
  int distance = mod1016.calculateDistance();
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");  
  }
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE DETECTED");
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        break;
      case 8: 
        Serial.println("LIGHTNING DETECTED");
        printDistance();
        break;
    }
}



void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("Welcome to the AS3935 Lightning Sensor test sketch!");
 

	// nur für playing with fusion board
  pinMode(CS_PIN, OUTPUT);
	digitalWrite(CS_PIN, HIGH);


  //I2C
  // setup for the the I2C library: (enable pullups, set speed to 400kHz)
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); 
  delay(2);

  // ???lightning0.AS3935_DefInit();   // set registers to default
  // lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
  mod1016.init(IRQ_PIN);

  //Tune Caps, Set AFE, Set Noise Floor
  autoTuneCaps(IRQ_PIN);
  //mod1016.setTuneCaps(7);
  mod1016.setOutdoors();
  mod1016.setNoiseFloor(5);
  
  
  Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");

  
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
 // mod1016.getIRQ();
  Serial.println("after interrupt");
}

void loop() {
  if (detected) {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }
}



