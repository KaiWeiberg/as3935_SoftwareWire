//	based on AS3935 MOD-1016 Lightning Sensor Arduino library by Embedded Adventures
//  Modified by Kai Weiberg
#include "AS3935.h"
#include "SoftwareWire.h"
#include "Arduino.h"
#include "FreqCount.h"

extern SoftwareWire I2CBUS;


AS3935* AS3935::getInstance() {                                                                                
    static AS3935 instance; 
    return &instance;                                                            
}   

void AS3935::init(uint8_t irqPin,void(*fZeiger)()) {
	_error = AS3935_ERR_NONE;
	_irqPin = irqPin;
    pinMode(_irqPin, INPUT);
	// reset();
	// setDivisionRatio(0);
	userISR = fZeiger;
	delay(20);	
    // setTuneCaps(3);
	// calibrateRCOs();
	attachInterrupt(digitalPinToInterrupt(_irqPin), (*userISR), RISING);

//setLCOFreqDiv16();	
	// Calibrierungssequenz
}


bool AS3935::errorTest() { 
	return (_error == AS3935_ERR_NONE) ? false : true;
}

void AS3935::errorMessage(bool clearError) {
	if (_error != 0) {
		Serial.println ("AS3935 Error:");
    	
		if (CHECK_BIT(_error, 3))
			Serial.println("\t I2C Bus");

		if (CHECK_BIT(_error, 4))
			Serial.println("\t LCO calibration failed");

		if (CHECK_BIT(_error, 5))
			Serial.println("\t SRCO calibration failed");

		if (CHECK_BIT(_error, 6))
			Serial.println("\t TRCO calibration failed");

		if (CHECK_BIT(_error, 7))
			Serial.println("\t wrong argument");

		if (clearError == true)
			errorClear();
	}
}

void AS3935::errorClear() { 
	_error = AS3935_ERR_NONE;
}


/*--------------------------------------------------------*/

 void AS3935::autoTuneCaps() {
	_error = 0;
	uint8_t fdiv = getDivisionRatio();
	Serial.println("Measuring frequency. Please wait...\n");
	freqPerTuneCaps(fdiv);
	recommendTuning();
	calibrateRCOs();
	getIRQ();
}

void AS3935::calibrateRCOs() {

	WireTwo.beginTransmission(AS3935_ADDR);
	WireTwo.write(0x3D); // Calibrates automatically the internal RC Oscillators
	WireTwo.write(0x96);
	WireTwo.endTransmission();
	delay(4);

    uint8_t trco = readRegister(TRCO_DONE);
	uint8_t srco = readRegister(SRCO_DONE);        
    if(trco == 0x00)
        _error = AS3935_ERR_TRCO;
    if(srco == 0x00)
        _error = AS3935_ERR_SRCO;
} 


uint32_t AS3935::getFreqLCO() {
	uint8_t fdiv = getDivisionRatio();
	uint32_t frq  = getFrequency(DISP_LCO, 0x80);
	attachInterrupt(digitalPinToInterrupt(_irqPin), (*userISR), RISING);
	frq = frq * fdiv;
	if (frq-500000>1750) {
		_error = _error | AS3935_ERR_LCO;
		Serial.println("LCO out of range");
	}		
	return frq;	
}	
	
uint32_t AS3935::getFreqTRCO() {
	uint32_t frq  = getFrequency(DISP_TRCO, 0x20);
	attachInterrupt(digitalPinToInterrupt(_irqPin), (*userISR), RISING);
	return frq;		
}

uint32_t AS3935::getFreqSRCO() {
	
	detachInterrupt(digitalPinToInterrupt(_irqPin));
	uint32_t frq;
	writeRegister(DISP_SRCO, 0x40);
	FreqCount.begin(1000); 
	while (FreqCount.available()==0);
    frq = FreqCount.read();
	FreqCount.end(); 
	writeRegister(DISP_SRCO, 0x00); 
	attachInterrupt(digitalPinToInterrupt(_irqPin), (*userISR), RISING);
	return frq;
}



inline void AS3935::pulseDetected() {   //ISR
	if (_displayingFrequency)
		_pulse++;
}

inline void AS3935::handle_isr() {
  AS3935::getInstance()->pulseDetected();
}


uint32_t AS3935::getFrequency(uint8_t reg, uint8_t mask, uint8_t val) {
	writeRegister(reg, mask, val);
	delay (60); // wait until the oscillator of the AS3933 is stable
	attachInterrupt(digitalPinToInterrupt(_irqPin), handle_isr, RISING);
	_displayingFrequency = true;
	_pulse = 0;
	delay(500);
	_displayingFrequency = false;
	detachInterrupt(digitalPinToInterrupt(_irqPin));
	writeRegister(reg, mask, 0x00);                        //TODO muss hier nicht die blitzanzeige hin?
	return (_pulse*2);
}

void AS3935::freqPerTuneCaps(uint8_t fdiv) {
	for (int i = 0; i < 16; i++) {
			setTuneCaps(i);
			delay(2);
			// Measure frequency in 1/2th of a second
			int32_t freq = (getFrequency(DISP_LCO, 0x80) * fdiv) - 500000; 
							Serial.print("fdiv");
				Serial.println(fdiv);
			_cap_frequencies[i] = (freq < 0) ? -freq : freq;
			Serial.print("TUNE CAPS = ");
			Serial.print(i);
			Serial.print("\tFrequency - 500k = ");
			Serial.println(_cap_frequencies[i]);
			Serial.println();
		}
}

void AS3935::recommendTuning() {
	int best = 0, next, current;
	for (int i = 1; i < 16; i++) {
		current = best;
		next = i;
		best = (_cap_frequencies[next] < _cap_frequencies[current]) ? next : best; 
	}

	Serial.print("Best value for TUNE_CAPS: ");
	Serial.println(best);
	setTuneCaps(best);
	
	if (_cap_frequencies[best]>1750) {
		_error = _error | AS3935_ERR_LCO;
		Serial.println("LCO calibration failed");
	}		
} 

/*--------------------------------------------------------*/

uint8_t AS3935::getIRQ() {
	delay(4);
	return readRegister(IRQ_TBL);
}

uint8_t AS3935::getLightDistance() {
	readRegister(LGHT_DIST);
	return readRegister(LGHT_DIST);		
}

uint8_t AS3935::getIntensity() {
	uint8_t lsb, msb, mmsb;
	lsb = readRegisterRaw(0x04);
	msb = readRegisterRaw(0x05);
	mmsb = readRegisterRaw(0x06);
	return (uint32_t)lsb | ((uint32_t)msb << 8) | ((uint32_t)(mmsb & 0x1F) << 16);
}

/*--------------------------------------------------------*/

void AS3935::PowerUp() {
    writeRegister(PWD, 0);
	delay(10);
    writeRegister(DISP_TRCO, (0x01 << 5));
	delay(2);
	writeRegister(DISP_TRCO, (0x00 << 5));
	delay(2);
	calibrateRCOs();
}

void AS3935::PowerDown() {
	writeRegister(PWD, 1);	
	delay(20); 				//don't reduce: otherwise an SRCO failure might show up
}

void AS3935::reset()
{
	WireTwo.beginTransmission(AS3935_ADDR);
	WireTwo.write(0x3C);   //PRESET_DEFAULT
	WireTwo.write(0x96);
	WireTwo.endTransmission();
	delay(2);
}

void AS3935::clearStats()
{
	writeRegister(CL_STAT, 1);
    delay(2);
    writeRegister(CL_STAT, 0);
    delay(2);
    writeRegister(CL_STAT, 1);
    delay(2);
}

void AS3935::enableDisturbers() {
	writeRegister(MASK_DST, (0 << 5));
	delay(2);
}

void AS3935::disableDisturbers() {
	writeRegister(MASK_DST, (1 << 5));
	delay(2);			
}

/*--------------------------------------------------------*/

uint8_t AS3935::getTuneCaps() {
	return readRegister(TUN_CAP);	
}

void AS3935::setTuneCaps(uint8_t val) {
	writeRegister(TUN_CAP, val);
	delay(2);
}

bool AS3935::getIndoors() {
    if (readRegister(AFE_GB) == INDOORS)
		return true;
	else
		return false;
}

void AS3935::setIndoors(bool val) {
	if (val)
		writeRegister(AFE_GB, INDOORS);
	else
		writeRegister(AFE_GB, OUTDOORS);	
}

uint8_t AS3935::getNoiseFloor() {
	return readRegister(NOISE_FLOOR >> 4);	
}

void AS3935::setNoiseFloor(uint8_t noise) {
	writeRegister(NOISE_FLOOR, noise << 4);
}

uint8_t AS3935::getWatchdogThreshold() {
	return readRegister(WDTH);	
}

void AS3935::setWatchdogThreshold(uint8_t val) {
	writeRegister(WDTH, val);
}

uint8_t AS3935::getSpikeRejection(){
	return readRegister(SREJ);		
}

void AS3935::setSpikeRejection(uint8_t val) {
	writeRegister(SREJ, val);
}

uint8_t AS3935::getMinStrikes() {
	switch (readRegister(MIN_LIGH)){
        case 0x00:
       		return  1;
        case 0x10:
       		return  5;
        case 0x20:
       		return  9;
        default:
       		return  16;
    }	 
}

void AS3935::setMinStrikes(uint8_t val) {
    // DESCRIPTION      This function sets the minmum strikes, see pg 22 of the datasheet for more info (#strikes in 17 min)
    // CONDITIONS    	Options are 1, 5, 9 or 16 strikes.
	if (val == 1 || val == 5 || val == 9 || val == 16) {
		writeRegister(0x02, 0b11001111, val);
    } else {
		_error = AS3935_ERR_PARAMETER;
	}
	delay(2);
}

/*--------------------------------------------------------
						PRIVATE
----------------------------------------------------------*/


AS3935::AS3935(){
	userISR = 0;
}

uint8_t AS3935::readRegisterRaw(uint8_t reg) {
		uint8_t result;
		WireTwo.beginTransmission(AS3935_ADDR);
		WireTwo.write(reg);
		_error = _error | WireTwo.endTransmission(false);
		WireTwo.requestFrom(AS3935_ADDR, 1);
		if (WireTwo.available()) {
			result = WireTwo.read();
		}
		return result;
}

uint8_t AS3935::readRegister(uint8_t reg, uint8_t mask) {
	uint8_t data = readRegisterRaw(reg);
	return (data & mask);
}


void AS3935::writeRegister(uint8_t reg, uint8_t mask, uint8_t data) {
		uint8_t currentReg = readRegisterRaw(reg);
		currentReg = currentReg & (~mask);
		data |= currentReg;

		WireTwo.beginTransmission(AS3935_ADDR);
		WireTwo.write(reg);
		WireTwo.write(data);
		_error = _error | WireTwo.endTransmission(); //alter fehlerwert darf nicht überschrieben werden
}


uint8_t AS3935::getDivisionRatio() {
	switch (readRegister(LCO_FDIV >> 6)){
        case 0:
       		return  16;
        case 1:
       		return  32;
        case 2:
       		return  64;
        default:
       		return  128;
    }	 
}

void AS3935::setDivisionRatio(uint8_t val) {     // 0 ->16,1->32,2-
	writeRegister(LCO_FDIV, val << 6);
}



