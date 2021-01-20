#ifndef _SPARKFUN_HAPTIC_DRIVER_DA7280_
#define _SPARKFUN_HAPTIC_DRIVER_DA7280_

#include <Wire.h>
#include <Arduino.h>

#define DEF_ADDR 0x4A
#define CHIP_REV 0xBA
#define ENABLE 0x01
#define DISABLE 0x00
#define I2C_ONLY_MODE  0x01
#define I2C_PWM_MODE 0x02
#define GPIO_MODE 0x03
#define LRA_TYPE 0x00
#define ERM_TYPE 0x01


enum REGISTERS {

  CHIP_REV_REG = 0x00, //whoami?
  
  IRQ_EVENT1 = 0x03, 
  IRQ_EVENT_WARN_DIAG = 0x04, 
  IRQ_EVENT_SEQ_DIAG = 0x05, 
  IRQ_STATUS1 = 0x06, 
  IRQ_MASK1 = 0x07, 

  CIF_I2C1 = 0x08, 

  FRQ_LRA_PER_H = 0x0A, 
  FRQ_LRA_PER_L = 0x0B, 

  ACTUATOR1 = 0x0C, 
  ACTUATOR2 = 0x0D, 
  ACTUATOR3 = 0x0E, 

  CALIB_V2I_H = 0x0F, 
  CALIB_V2I_L = 0x10, 

  CALIB_IMP_H = 0x11, 
  CALIB_IMP_L = 0x12, 

  TOP_CFG1 = 0x13,
  TOP_CFG2 = 0x14,
  TOP_CFG3 = 0x15,
  TOP_CFG4 = 0x16,

  TOP_INT_CFG1 = 0x17,
  TOP_INT_CFG6_H = 0x1C,
  TOP_INT_CFG6_L = 0x1D,
  TOP_INT_CFG7_H = 0x1E,
  TOP_INT_CFG7_L = 0x1F,
  TOP_INT_CFG8 = 0x20,

  TOP_CTL1 = 0x22,
  TOP_CTL2 = 0x23,
  SEG_CTL1 = 0x24,

  SWG_C1 = 0x25,
  SWG_C2 = 0x26,
  SWG_C3 = 0x27,
  SEQ_CTL2 = 0x28,

  GPI_0_CTL = 0x29,
  GPI_1_CTL = 0x2A,
  GPI_2_CTL = 0x2B,

  MEM_CTL1 = 0x2C,
  MEM_CTL2 = 0x2D,
  
  ADC_DATA_H1 = 0x2E,
  ADC_DATA_L1 = 0x2F,

  POLARITY = 0x43,
  LRA_AVR_H = 0x44,
  LRA_AVR_L = 0x45,

  FRQ_LRA_PER_ACT_H = 0x46,
  FRQ_LRA_PER_ACT_L = 0x47,

  FRQ_PHASE_H = 0x48,
  FRQ_PHASE_L = 0x49,
  FRQ_CTL = 0x4C,

  TRIM3 = 0x5F,
  TRIM4 = 0x60,
  TRIM6 = 0x62,

  TOP_CFG5 = 0x62,
  IRQ_EVENT_ACTUATOR_FAULT = 0x81,
  IRQ_STATUS2 = 0x82,
  IRQ_MASK2 = 0x83,
  SNP_MEM_X = 0x84 // to 0xE7

};

enum BIT_POSITIONS {

  POS_ZERO = 0x00, 
  POS_ONE, 
  POS_TWO, 
  POS_THREE, 
  POS_FOUR, 
  POS_FIVE,
  POS_SIX, 
  POS_SEVEN

};

enum BIT_POS_MASKS {

  BIT_POS_ZERO = 0xFE,
  BIT_POS_ONE = 0xFD,
  BIT_POS_TWO = 0xFB,
  BIT_POS_THREE = 0xF7,
  BIT_POS_FOUR = 0xEF,
  BIT_POS_FIVE = 0xDF,
  BIT_POS_SIX = 0xBF,
  BIT_POS_SEVEN = 0x7F,
  I2C_WR_MASK = 0x7F

};

enum BIT_VAL_MASKS {

  BIT_VAL_ZERO = 0x00,
  BIT_VAL_ONE,
  BIT_VAL_TWO,
  BIT_VAL_THREE,
  BIT_VAL_FOUR,
  BIT_VAL_FIVE,
  BIT_VAL_SIX,
  BIT_VAL_SEVEN,
  BIT_VAL_EIGHT,
  BIT_VAL_NINE,
  BIT_VAL_TEN,
  BIT_VAL_ELEVEN,
  BIT_VAL_TWELVE,
  BIT_VAL_THIRT,
  BIT_VAL_FOURT,
  BIT_VAL_FIFT,
  BIT_VAL_7F = 0x7F,
  BIT_VAL_FF = 0xFF,
  BIT_VAL_MSB_F = 0xF00

};

class Haptic_Driver
{  
  public:
    
    // Public Variables
    
    //Function declarations
    Haptic_Driver(uint8_t address = DEF_ADDR); // I2C Constructor

    bool begin(TwoWire &wirePort = Wire); // begin function

    bool setActuatorType(uint8_t);
    bool setOperationMode(uint8_t);
    bool writeI2CWave(uint8_t);

    bool setDefaultSettings(uint8_t soundMode = I2C_ONLY_MODE);
    bool setActuatorABSVolt(float);
    bool setActuatorNOMVolt(float);
    bool setActuatorIMAX(float);
    bool setActuatorImpedance(float);
    bool setActuatorLRAfrequency(float);
    bool enableCoinERM();
    bool enableAcceleration(bool);
    bool enableRapidStop(bool);
    bool enableAmpPid(bool);
    bool setBemfFaultLimit(bool);
    bool enableV2iFactorFreeze(bool);
    bool calibrateImpedanceDistance(bool);
    bool setVibrateVal(uint8_t);

    uint8_t _readRegister(uint8_t);
  private:
    
    // Private Variables
    uint8_t _address;

    // This generic function handles I2C write commands for modifying individual
    // bits in an eight bit register. Paramaters include the register's address, a mask 
    // for bits that are ignored, the bits to write, and the bits' starting
    // position.
    bool _writeRegister(uint8_t, uint8_t, uint8_t, uint8_t);

    // Consecutive Write Mode: I2C_WR_MODE = 0
    // Allows for n-number of writes on consecutive registers, beginning at the
    // given register. 
    // This particular write does not care what is currently in the register and
    // overwrites whatever is there.
    uint8_t _writeConsReg(uint8_t regs[], size_t);

    // Non-Consecutive Write Mode: I2C_WR_MODE = 1
    // Allows for n-number of writes on non-consecutive registers, beginning at the
    // given register but able to jump locations by giving another address. 
    // This particular write does not care what is currently in the register and
    // overwrites whatever is there.
    uint8_t _writeNonConsReg(uint8_t regs[], size_t);

    // This generic function does a basic I-squared-C write transaction at the
    // given address, and writes the given _command argument. 
    void _writeCommand(uint8_t);

    // This generic function reads an eight bit register. It takes the register's
    // address as its' parameter. 

    bool _readConsReg(uint8_t regs[], size_t);
    bool _readNonConsReg(uint8_t regs[], size_t);
    // This generic function does a basic I-squared-C read transaction at the given
    // addres, taking the number of reads as argument. 
    uint8_t _readCommand(uint8_t);


    TwoWire *_i2cPort;
};
#endif
