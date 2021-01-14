/*
  This is a library for...
  By: Elias Santistevan
  Date: 
  License: This code is public domain but you buy me a beer if you use this and 
  we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
 */

#include "Haptic_Driver.h"

Haptic_Driver::Haptic_Driver(uint8_t address){  _address = address; } //Constructor for I2C

bool Haptic_Driver::begin( TwoWire &wirePort )
{
  
  _i2cPort = &wirePort;
  uint8_t chipRev;

  uint8_t ret = _readRegister(CHIP_REV_REG); 
  chipRev |= ret << 8;
  chipRev |= ret; 
  Serial.println(chipRev, HEX);

  if( chipRev != CHIP_REV )
    return false;
  else
    return true; 

}


bool Haptic_Driver::setActuator(uint8_t actuator){

  if( actuator != 0 || actuator != 1 )
    return false; 

  if( _writeRegister( TOP_CFG1, 0b11011111, actuator, POS_FIVE ) )
    return true; 
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

  if( _i2cPort->endTransmission() ) // End communcation.
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
