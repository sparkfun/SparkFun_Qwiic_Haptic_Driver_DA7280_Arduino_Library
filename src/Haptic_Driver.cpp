/*
  This is a library for...
  By: Elias Santistevan
  Date: 
  License: This code is public domain but you buy me a beer if you use this and 
  we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
 */

#include "Haptic_Driver.h"
#include "math.h"

Haptic_Driver::Haptic_Driver(uint8_t address){  _address = address; } //Constructor for I2C

bool Haptic_Driver::begin( TwoWire &wirePort )
{
  
  _i2cPort = &wirePort;
  uint8_t chipRev;

  uint8_t ret = _readRegister(CHIP_REV_REG); 
  chipRev |= ret << 8;
  chipRev |= ret; 

  if( chipRev != CHIP_REV )
    return false;
  else
    return true; 

}

bool Haptic_Driver::setActuatorType(uint8_t actuator){
  
  if( actuator < 0 || actuator > 1 )
    return false; 

  if( _writeRegister( TOP_CFG1, BIT_POS_FIVE, actuator, POS_FIVE ) )
    return true; 
  else
    return false; 
}

bool Haptic_Driver::setOperationMode(uint8_t mode){ 

  if( mode < 0 || mode > 3) 
    return false;

  if( _writeRegister(TOP_CTL1, BIT_VAL_SEVEN, mode, POS_ZERO) ) 
    return true; 
  else
    return false; 

}

bool Haptic_Driver::writeI2CWave(uint8_t wave){

  uint8_t accelState = _readRegister(TOP_CFG1);
  accelState &= BIT_VAL_FOUR; 
  accelState = accelState >> POS_TWO;  
  Serial.println("IRQ_EVENT1:");
  Serial.println(_readRegister(IRQ_EVENT1), BIN);
  Serial.println("IRQ_EVENT_WARN_DIAG:");
  Serial.println(_readRegister(IRQ_EVENT_WARN_DIAG), BIN);
  Serial.println("IRQ_STATUS1:");
  Serial.println(_readRegister(IRQ_STATUS1), BIN);

  if( accelState == ENABLE ){
    if( wave < 0x00 || wave > 0x7F ) 
      return false;
  }
  else {
    if( wave < 0x00 || wave > 0xFF ) 
      return false;
  }

  if( _writeRegister(TOP_CTL2, BIT_VAL_ZERO, wave, POS_ZERO) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setDefaultSettings(uint8_t soundMode){

  if( setActuatorType(LRA_TYPE) &&\ 
      setActuatorABSVolt(2.5) &&\
      setActuatorNOMVolt(2.5) &&\
      setActuatorIMAX(170) &&\
      setActuatorImpedance(13.8) &&\
      setActuatorLRAfrequency(170) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setActuatorABSVolt(float absVolt){

  if( absVolt < 0 || absVolt > 3.3)
    return false; 
  
  absVolt = absVolt/(23.4 * pow(10,-3)); 

  if( _writeRegister(ACTUATOR2, BIT_VAL_ZERO, static_cast<uint8_t>(absVolt), POS_ZERO) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setActuatorNOMVolt(float rmsVolt){

  if( rmsVolt < 0 || rmsVolt > 3.3 )
    return false; 

  rmsVolt = rmsVolt/(23.4 * pow(10,-3)); 
    
  if( _writeRegister(ACTUATOR1 , BIT_VAL_ZERO, static_cast<uint8_t>(rmsVolt), POS_ZERO ) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setActuatorIMAX(float maxCurr){

  if( maxCurr < 0 || maxCurr > 300.0) // Random upper limit - FIX 
    return false; 
  
  maxCurr = (maxCurr - 28.6)/7.2;

  if( _writeRegister(ACTUATOR3, BIT_VAL_ZERO, static_cast<uint8_t>(maxCurr), POS_ZERO) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setActuatorImpedance(float motorImpedance){

  if( motorImpedance < 0 || motorImpedance > 500.0) // Random upper limit - FIX
    return false; 

  uint8_t msbImpedance; 
  uint8_t lsbImpedance;
  uint16_t v2iFactor;
  uint8_t maxCurr = _readRegister(ACTUATOR3) | BIT_VAL_FIFT;

  v2iFactor = (motorImpedance * (maxCurr + 4))/1.6104;
  msbImpedance = (v2iFactor - (v2iFactor & BIT_VAL_FF))/256;
  lsbImpedance = (v2iFactor - (256 * (v2iFactor & BIT_VAL_MSB_F)));

  if( _writeRegister(CALIB_V2I_L, BIT_VAL_ZERO, lsbImpedance, POS_ZERO) &&\
      _writeRegister(CALIB_V2I_H, BIT_VAL_ZERO, msbImpedance, POS_ZERO) )
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::setActuatorLRAfrequency(float frequency){


  if( frequency < 0 || frequency > 500.0 )
    return false; 
  
  uint8_t msbFrequency;
  uint8_t lsbFrequency;
  uint16_t lraPeriod;

  lraPeriod = 1/(frequency * (1333.32 * pow(10, -9)));
  msbFrequency = (lraPeriod - (lraPeriod & BIT_VAL_7F))/128;
  lsbFrequency = (lraPeriod - 128 * (lraPeriod & BIT_VAL_MSB_F));

  if( _writeRegister(FRQ_LRA_PER_H, BIT_VAL_ZERO, msbFrequency, POS_ZERO) &&\ 
      _writeRegister(FRQ_LRA_PER_L, BIT_VAL_ZERO, lsbFrequency, POS_ZERO) ){
    return true;
  }
  else
    return false; 
  
}

bool Haptic_Driver::enableCoinERM(){

  if( enableAcceleration(false) &&\
      enableRapidStop(false) &&\ 
      enableAmpPid(false) &&\ 
      enableV2iFactorFreeze(true) &&\
      calibrateImpedanceDistance(true) &&\
      setBemfFaultLimit(true) ) 
    return true;
  else
    return false; 
  
}

bool Haptic_Driver::enableAcceleration(bool enable){

  if( _writeRegister(TOP_CFG1, BIT_POS_TWO, static_cast<uint8_t>(enable), POS_TWO) )
    return true;
  else
    return false; 
}


bool Haptic_Driver::enableRapidStop(bool enable){

  if( _writeRegister(TOP_CFG1, BIT_POS_ONE, static_cast<uint8_t>(enable), POS_ONE) )
    return true;
  else
    return false; 
}

bool Haptic_Driver::enableAmpPid(bool enable){

  if( _writeRegister(TOP_CFG1, BIT_POS_ZERO, static_cast<uint8_t>(enable), POS_ZERO) )
    return true;
  else
    return false; 
}

bool Haptic_Driver::setBemfFaultLimit(bool enable){

  if( _writeRegister(TOP_CFG1, BIT_POS_FOUR, static_cast<uint8_t>(enable), POS_FOUR) )
    return true;
  else
    return false; 
}

bool Haptic_Driver::enableV2iFactorFreeze(bool enable){

  if( _writeRegister(TOP_CFG4, BIT_POS_SEVEN, static_cast<uint8_t>(enable), POS_SEVEN) )
    return true;
  else
    return false; 
}


bool Haptic_Driver::calibrateImpedanceDistance(bool enable){

  if( _writeRegister(TOP_CFG4, BIT_POS_SIX, static_cast<uint8_t>(enable), POS_SIX) )
    return true;
  else
    return false; 
}

bool Haptic_Driver::setVibrateVal(uint8_t val){
  if( val < 0 || val > 255 )
    return false; 

  if( _writeRegister(TOP_CTL2, BIT_VAL_ZERO, val, POS_ZERO) )
    return true;
  else
    return false; 
  
}


// This generic function handles I2C write commands for modifying individual
// bits in an eight bit register. Paramaters include the register's address, a mask 
// for bits that are ignored, the bits to write, and the bits' starting
// position.
bool Haptic_Driver::_writeRegister(uint8_t _wReg, uint8_t _mask, uint8_t _bits, uint8_t _startPosition)
{

  uint8_t _i2cWrite;
  _i2cWrite = _readRegister(_wReg); // Get the current value of the register
  _i2cWrite &= (_mask); // Mask the position we want to write to.
  _i2cWrite |= (_bits << _startPosition);  // Write the given bits to the variable
  _i2cPort->beginTransmission(_address); // Start communication.
  _i2cPort->write(_wReg); // at register....
  _i2cPort->write(_i2cWrite); // Write register...

  if( !_i2cPort->endTransmission() ) // End communcation.
    return true; 
  else 
    return false; 
  
}

// This generic function does a basic I-squared-C write transaction at the
// given address, and writes the given _command argument. 
void Haptic_Driver::_writeCommand(uint8_t _command){

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_command);
    _i2cPort->endTransmission(); 

}

// This generic function reads an eight bit register. It takes the register's
// address as its' parameter. 
uint8_t Haptic_Driver::_readRegister(uint8_t _reg)
{

  _i2cPort->beginTransmission(_address); 
  _i2cPort->write(_reg); // Moves pointer to register.
  _i2cPort->endTransmission(false); // 'False' here sends a re-"start" message so that bus is not released
  _i2cPort->requestFrom(_address, 1); // Read the register, only ever once. 
  uint8_t _regValue = _i2cPort->read();
  return(_regValue);
}

// Consecutive Read Mode: I2C_WR_MODE = 0
// Allows for n-number of reads on consecutive registers, beginning at the
// given register. 
bool Haptic_Driver::_readConsReg(uint8_t reg[], size_t _numReads){
  return true;
}

// Non-Consecutive Read Mode: I2C_WR_MODE = 1
// Allows for n-number of reads on non-consecutive registers, beginning at the
// given register but able to jump locations by giving another address. 
bool Haptic_Driver::_readNonConsReg(uint8_t reg[], size_t numReads){
  return true; 
}

// Consecutive Write Mode: I2C_WR_MODE = 0
// Allows for n-number of writes on consecutive registers, beginning at the
// given register. 
// This particular write does not care what is currently in the register and
// overwrites whatever is there.
uint8_t Haptic_Driver::_writeConsReg(uint8_t regs[], size_t numWrites){

  _writeRegister(CIF_I2C1, I2C_WR_MASK, 0, 0x07);      

  _i2cPort->beginTransmission(_address);

  for( size_t i = 0; i <= numWrites; i++){
      _i2cPort->write(regs[i]);
  }

  if(!_i2cPort->endTransmission())
    return true;
  else
    return false;

}

// Non-Consecutive Write Mode: I2C_WR_MODE = 1
// Allows for n-number of writes on non-consecutive registers, beginning at the
// given register but able to jump locations by giving another address. 
// This particular write does not care what is currently in the register and
// overwrites whatever is there.
uint8_t Haptic_Driver::_writeNonConsReg(uint8_t regs[], size_t numWrites){

  _writeRegister(CIF_I2C1, I2C_WR_MASK, 1, 0x07);      

  _i2cPort->beginTransmission(_address); // Start communication.
  for( size_t i = 0; i <= numWrites; i++){
    // Here's to hoping that the register pointer will indeed jump locations as
    // advertised.
    _i2cPort->write(regs[i]);
  }

  if(!_i2cPort->endTransmission())
    return true;
  else
    return false;

}



// This generic function does a basic I-squared-C read transaction at the given
// address, taking the number of reads as argument. 
uint8_t Haptic_Driver::_readCommand(uint8_t _numReads)
{

  _i2cPort->requestFrom(_address, _numReads);  
  uint8_t someVal = _i2cPort->read();
  return(someVal);

}
