#ifndef _SPARKFUN_HAPTIC_DRIVER_DA7280_
#define _SPARKFUN_HAPTIC_DRIVER_DA7280_

#include <Wire.h>
#include <Arduino.h>

#define DEF_ADDR 0x4A
#define CHIP_REV 0xAB
#define 

enum {
  CHIP_REV_REG = 0x00,
  IRQ_EVENT1 = 0x03, 
  IRQ_EVENT_WARN_DIAG = 0x04, 
  IRQ_EVENT_SEQ_DIAG = 0x05, 
  IRQ_STATUS1 = 0x06, 
  IRQ_MASK1 = 0x07, 
  CIF_I2C1 = 0x08, 
  FRQ_LRA_PER_H = 0x0A, 
};

enum {
  I2C_WR_MASK = 0x7F 
};

class Haptic_Driver
{  
  public:
    
    // Public Variables
    
    //Function declarations
    Haptic_Driver(uint8_t); // I2C Constructor

    bool begin(TwoWire &wirePort = Wire); // begin function

  private:
    
    // Private Variables
    uint8_t _address;

    // This generic function handles I2C write commands for modifying individual
    // bits in an eight bit register. Paramaters include the register's address, a mask 
    // for bits that are ignored, the bits to write, and the bits' starting
    // position.
    void _writeRegister(uint8_t, uint8_t, uint8_t, uint8_t);

    // Consecutive Write Mode: I2C_WR_MODE = 0
    // Allows for n-number of writes on consecutive registers, beginning at the
    // given register. 
    // This particular write does not care what is currently in the register and
    // overwrites whatever is there.
    uint8_t _writeConsReg(uint8_t*);

    // Non-Consecutive Write Mode: I2C_WR_MODE = 1
    // Allows for n-number of writes on non-consecutive registers, beginning at the
    // given register but able to jump locations by giving another address. 
    // This particular write does not care what is currently in the register and
    // overwrites whatever is there.
    uint8_t _writeNonConsReg(uint8_t*);

    // This generic function does a basic I-squared-C write transaction at the
    // given address, and writes the given _command argument. 
    void _writeCommand(uint8_t);

    // This generic function reads an eight bit register. It takes the register's
    // address as its' parameter. 
    uint8_t _readRegister(uint8_t);

    uint8_t _readConsReg(uint8_t, uint8_t);
    uint8_t _readNonConsReg(uint8_t);
    // This generic function does a basic I-squared-C read transaction at the given
    // addres, taking the number of reads as argument. 
    uint8_t _readCommand(uint8_t);


    TwoWire *_i2cPort;
};
#endif
