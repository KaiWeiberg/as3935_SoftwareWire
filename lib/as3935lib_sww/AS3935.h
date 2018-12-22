
//	AS3935 MOD-1016 Lightning Sensor Arduino library
//	Written originally by Embedded Adventures
//  modified by Kai Weiberg

#ifndef __AS3935_h
#define __AS3935_h

#define I2CBUS WireTwo

#include <Arduino.h>
#include <AsyncDelay.h>
#include "SoftwareWire.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define AS3935_ADDR 0x03
#define INDOORS 0x24   // 36
#define OUTDOORS 0x1C  // 

//		     Reg address	bitmask
#define AFE_GB 0x00, 0x3E
#define PWD 0x00, 0x01
#define NOISE_FLOOR 0x01, 0x70
#define WDTH 0x01, 0x0F
#define CL_STAT 0x02, 0x40
#define MIN_LIGH 0x02, 0x30
#define SREJ 0x02, 0x0F
#define LCO_FDIV 0x03, 0xC0
#define MASK_DST 0x03, 0x20
#define IRQ_TBL 0x03, 0x0F
#define LGHT_DIST 0x07, 0x3F
#define DISP_LCO 0x08, 0x80
#define DISP_SRCO 0x08, 0x40
#define DISP_TRCO 0x08, 0x20
#define TUN_CAP 0x08, 0x0F


#define TRCO_FAIL 0x3A, 0x40  //TODO 
#define TRCO_DONE 0x3A, 0x80 
#define SRCO_FAIL 0x3B, 0x40 
#define SRCO_DONE 0x3B, 0x80 


#define AS3935_ERR_NONE 		0               
#define AS3935_ERR_I2C			8
#define AS3935_ERR_LCO  		16   			/* LCO  500khz calibration failed or LCO out of range  */                    
#define AS3935_ERR_SRCO			32   			/* SRCO calibration failed */
#define AS3935_ERR_TRCO 		64   			/* TRCO calibration failed */
#define AS3935_ERR_PARAMETER  	128   		  


class AS3935
{
  private:
	uint8_t _irqPin;
	uint8_t _error;
    volatile uint16_t _pulse;
	volatile bool _displayingFrequency;
  	int32_t _cap_frequencies[16];
    void (*userISR)();

    AS3935();
	uint8_t readRegisterRaw(uint8_t reg);    
	uint32_t getFrequency(uint8_t reg, uint8_t mask, uint8_t val);   // Generate the frequency output and measure number of pulses	
	void freqPerTuneCaps(uint8_t fdiv); 	//Measure frequency output under setting 0-15 on TUNE_CAPS
	void recommendTuning();  				//Search for the tuning whose frequency is closest to 500kHz
	void writeRegister(uint8_t reg, uint8_t mask, uint8_t data);
	uint8_t readRegister(uint8_t reg, uint8_t mask);  //, uint8_t &data
	inline void pulseDetected(); 			//ISR for measuring the frequency output on the IRQ pin
	inline static void handle_isr();

  public:
	static AS3935* getInstance();
	void init(uint8_t irqPin, void (*fZeiger)());
	
	bool errorTest();  // 'true' if an error happens
	void errorMessage(bool clearError);
	void errorClear();

	void autoTuneCaps();	
	void calibrateRCOs();
	uint32_t getFreqLCO();	
	uint32_t getFreqSRCO(); //Arduino Uno can only measure up to 1MHz
	uint32_t getFreqTRCO();
	void setDivisionRatio(uint8_t val);
	uint8_t getDivisionRatio();


	uint8_t getIRQ();
	uint8_t getLightDistance(); // Result: km or 0x3F = Out of Range,
	uint8_t getIntensity();   //TODO: reicht 16?
	
	void PowerUp();
	void PowerDown();
	void reset();
	void clearStats();
	void enableDisturbers();
	void disableDisturbers();

	void setIndoors(bool val);
	void setTuneCaps(uint8_t val);     		// values between 0 and 15, default 2
	void setNoiseFloor(uint8_t val);  		// values between 0 and 7, default 2
	void setWatchdogThreshold(uint8_t val); // between 0 and 10 ?? (15), default 2. Increase robustness to disturbers
	void setSpikeRejection(uint8_t val); 	// between 0 and 10 ?? (15), default 2
	void setMinStrikes(uint8_t val); 		// Options are 1, 5, 9 or 16 strikes, default 1

	uint8_t getTuneCaps();
	bool getIndoors();
	uint8_t getNoiseFloor();  	
	uint8_t getWatchdogThreshold();
	uint8_t getSpikeRejection();
	uint8_t getMinStrikes();
};


#endif
