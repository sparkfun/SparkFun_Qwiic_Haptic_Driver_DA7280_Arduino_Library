#ifndef _SPARKFUN_HAPTIC_DRIVER_DA7280_
#define _SPARKFUN_HAPTIC_DRIVER_DA7280_

#include <Wire.h>
#include <Arduino.h>

#define DEF_ADDR 0x4A
#define CHIP_REV 0xBA
#define ENABLE 0x01
#define UNLOCKED 0x01
#define DISABLE 0x00
#define LOCKED 0x00
#define LRA_TYPE 0x00
#define ERM_TYPE 0x01

enum OPERATION_MODES {

   INACTIVE  = 0x00, 
   DRO_MODE, 
   PWM_MODE,
   RTWM_MODE, 
   ETWM_MODE

};

enum REGISTERS {

  CHIP_REV_REG = 0x00, //whoami?
  
  IRQ_EVENT1 = 0x03, 
  IRQ_EVENT_WARN_DIAG, 
  IRQ_EVENT_SEQ_DIAG, 
  IRQ_STATUS1, 
  IRQ_MASK1, 

  CIF_I2C1, 

  FRQ_LRA_PER_H = 0x0A, 
  FRQ_LRA_PER_L, 

  ACTUATOR1, 
  ACTUATOR2, 
  ACTUATOR3, 

  CALIB_V2I_H, 
  CALIB_V2I_L = 0x10, 

  CALIB_IMP_H, 
  CALIB_IMP_L, 

  TOP_CFG1,
  TOP_CFG2,
  TOP_CFG3,
  TOP_CFG4,

  TOP_INT_CFG1,
  TOP_INT_CFG6_H = 0x1C,
  TOP_INT_CFG6_L,
  TOP_INT_CFG7_H,
  TOP_INT_CFG7_L,
  TOP_INT_CFG8 = 0x20,

  TOP_CTL1 = 0x22,
  TOP_CTL2,
  SEG_CTL1,

  SWG_C1,
  SWG_C2,
  SWG_C3,
  SEQ_CTL2,

  GPI_0_CTL,
  GPI_1_CTL,
  GPI_2_CTL,

  MEM_CTL1,
  MEM_CTL2,
  
  ADC_DATA_H1,
  ADC_DATA_L1,

  POLARITY = 0x43,
  LRA_AVR_H,
  LRA_AVR_L,

  FRQ_LRA_PER_ACT_H,
  FRQ_LRA_PER_ACT_L,

  FRQ_PHASE_H,
  FRQ_PHASE_L,
  FRQ_CTL = 0x4C,

  TRIM3 = 0x5F,
  TRIM4,
  TRIM6 = 0x62,

  TOP_CFG5 = 0x6E,
  IRQ_EVENT_ACTUATOR_FAULT = 0x81,
  IRQ_STATUS2,
  IRQ_MASK2,
  SNP_MEM_X

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

enum IRQ_EVENTS {

  E_SEQ_CONTINUE = 0x01
  E_UVLO = 0x02,
  E_SEQ_DONE = 0x04,
  E_OVERTEMP_CRIT = 0x08,
  E_SEQ_FAULT = 0x10,
  E_WARNING = 0x20,
  E_ACTUATOR_FAULT = 0x40,
  E_OC_VAULT = 0x80

};

class Haptic_Driver
{  
  public:
    
    // Public Variables
    uint8_t numOfSnippets = 0; 
    
    //Function declarations
    Haptic_Driver(uint8_t address = DEF_ADDR); // I2C Constructor

    bool begin(TwoWire &wirePort = Wire); // begin function

    bool setActuatorType(uint8_t);
    bool setOperationMode(uint8_t);
    bool writeI2CWave(uint8_t);

    bool setDefaultSettings(uint8_t soundMode = DRO_MODE);
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
    bool waveFormSettings(uint8_t);

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
    uint8_t _readRegister(uint8_t);

    bool _readConsReg(uint8_t regs[], size_t);
    bool _readNonConsReg(uint8_t regs[], size_t);
    // This generic function does a basic I-squared-C read transaction at the given
    // addres, taking the number of reads as argument. 
    uint8_t _readCommand(uint8_t);


    TwoWire *_i2cPort;
};
#endif
