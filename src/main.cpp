#include <Arduino.h>
#include <AS3935.h>
#include "SoftwareWire.h"

#define CS_PIN 9

const uint8_t IRQ_pin = 3;
const uint8_t SCL_pin = 22;   
const uint8_t SDA_pin = 23; 

// const uint8_t SCL_pin = 21;   
// const uint8_t SDA_pin = 20; 


SoftwareWire WireTwo(SDA_pin, SCL_pin, true,true);

AS3935* mod1016 = AS3935::getInstance();

volatile bool detected = false;

void alert() {
  detected = true;
}

void test() {
      Serial.println("test\n");
}

void printDistance() {
  uint8_t distance =mod1016->getLightDistance();
  if (distance == 0x3F)
    Serial.println("Lightning out of range");
  else if (distance == 5)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");  
  }
}

void translateIRQ(uint8_t irq) {
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
  Serial.println();
  Serial.println("AS3935 Lightning Sensor test sketch!");

  pinMode(CS_PIN, OUTPUT);
	digitalWrite(CS_PIN, HIGH);


  void (*zeiger)() = 0;
  zeiger = &test;
  zeiger();

  delay(2);
  WireTwo.begin();   

  mod1016->init(IRQ_pin, alert);

  // delay(100);
  mod1016->setTuneCaps(6);
  mod1016->setIndoors(true);
  mod1016->setNoiseFloor(2);
  //mod1016->autoTuneCaps();

   Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
   Serial.print(mod1016->getTuneCaps(), HEX);
   Serial.print("\t");
   Serial.print(mod1016->getIndoors());
   Serial.print("\t");
   Serial.println( mod1016->getNoiseFloor(), HEX);
   Serial.print("\n");
  
   mod1016->PowerDown();
   mod1016->PowerUp();
     mod1016->calibrateRCOs();

  Serial.println(mod1016->getFreqTRCO()/1000.00);
  Serial.println(mod1016->getFreqLCO()/1000.00);
  Serial.println(mod1016->getFreqSRCO()/1000.00);

  Serial.print("irq ");
   Serial.println( mod1016->getIRQ(), HEX);

  mod1016->getIRQ();      //wann muss während des setups der irq ausgelesen werden?
  if (mod1016->errorTest()){
    mod1016->errorMessage(false);
    mod1016->errorClear();
  } else {
    Serial.println("No errors");
  }
    mod1016->getIRQ();      //wann muss während des setups der irq ausgelesen werden?
}

void loop() {
  if (detected) {
    translateIRQ(mod1016->getIRQ());
    detected = false;
  }
  delay(1000);
}