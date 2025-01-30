/**
 * University of Washington - Bothell
 * NNM Capstone Project 2022-2023
 * 
 * @authors Adam Gibson, Joel Elliot
 * 
 * Purpose:
 * Provide a starting point for the combined code structure
 * (Arduino, MegunoLink, and AFE register configuration)
 * 
 * Hardware Setup:
 * Arduino Uno connected to an Analog Devices ADPD105 AFE
 * via an I2C converter and a 5V to 1.8V level converter for the GPIO line
 * 
 * Last revised and modified by David J. Kim on 1/29/25
 */

// BEGIN - DEFINE CONSTANTS**********************************************
// I2C slave address of AFE chip
// single-byte (8-bit)
#define AFE_I2C_ADDRESS   0x64

// arduino gpio pin for AFE interrupt signal when
// data is ready to be read from the AFE data registers
#define AFE_DATA_READY    2

// AFE configuration register addresses
// all addresses are single-byte (8-bit)
#define AFE_STATUS_REG                  0x00
#define AFE_INT_MASK_REG                0x01
#define AFE_GPIO_DRV_REG                0x02
#define AFE_FIFO_THRESH_REG             0x06
#define AFE_DEVID_REG                   0x08
#define AFE_I2C_ID_REG                  0x09
#define AFE_CLK_RATIO_REG               0x0A
#define AFE_GPIO_CTRL_REG               0x0B
#define AFE_SLAVE_ADDRESS_KEY_REG       0x0D
#define AFE_SW_RESET_REG                0x0F
#define AFE_MODE_REG                    0x10
#define AFE_SLOT_EN_REG                 0x11
#define AFE_FSAMPLE_REG                 0x12
#define AFE_PD_LED_SELECT_REG           0x14
#define AFE_NUM_AVG_REG                 0x15
#define AFE_SLOTA_CH1_OFFSET_REG        0x18
#define AFE_SLOTA_CH2_OFFSET_REG        0x19
#define AFE_SLOTA_CH3_OFFSET_REG        0x1A
#define AFE_SLOTA_CH4_OFFSET_REG        0x1B
#define AFE_SLOTB_CH1_OFFSET_REG        0x1E
#define AFE_SLOTB_CH2_OFFSET_REG        0x1F
#define AFE_SLOTB_CH3_OFFSET_REG        0x20
#define AFE_SLOTB_CH4_OFFSET_REG        0x21
#define AFE_ILED3_COARSE_REG            0x22
#define AFE_ILED1_COARSE_REG            0x23
#define AFE_ILED2_COARSE_REG            0x24
#define AFE_ILED_FINE_REG               0x25
#define AFE_SLOTA_LED_PULSE_REG         0x30
#define AFE_SLOTA_NUMPULSES_REG         0x31
#define AFE_LED_DISABLE_REG             0x34
#define AFE_SLOTB_LED_PULSE_REG         0x35
#define AFE_SLOTB_NUMPULSES_REG         0x36
#define AFE_ALT_PWR_DN_REG              0x37
#define AFE_EXT_SYNC_STARTUP_REG        0x38
#define AFE_SLOTA_AFE_WINDOW_REG        0x39
#define AFE_SLOTB_AFE_WINDOW_REG        0x3B
#define AFE_AFE_PWR_CFG1_REG            0x3C
#define AFE_SLOTA_TIA_CFG_REG           0x42
#define AFE_SLOTA_AFE_CFG_REG           0x43
#define AFE_SLOTB_TIA_CFG_REG           0x44
#define AFE_SLOTB_AFE_CFG_REG           0x45
#define AFE_SAMPLE_CLK_REG              0x4B
#define AFE_CLK32M_ADJUST_REG           0x4D
#define AFE_ADC_CLOCK_REG               0x4E
#define AFE_EXT_SYNC_SEL_REG            0x4F
#define AFE_CLK_32M_CAL_EN_REG          0x50
#define AFE_PWR_CFG2_REG                0x54
#define AFE_TIA_INDEP_GAIN_REG          0x55
#define AFE_DIGITAL_INT_EN_REG          0x58
#define AFE_DIG_INT_CFG_REG             0x5A
#define AFE_DATA_ACCESS_CTL_REG         0x5F
#define AFE_FIFO_ACCESS_REG             0x60
#define AFE_SLOTA_PD1_16BIT_REG         0x64
#define AFE_SLOTA_PD2_16BIT_REG         0x65
#define AFE_SLOTA_PD3_16BIT_REG         0x66
#define AFE_SLOTA_PD4_16BIT_REG         0x67
#define AFE_SLOTB_PD1_16BIT_REG         0x68
#define AFE_SLOTB_PD2_16BIT_REG         0x69
#define AFE_SLOTB_PD3_16BIT_REG         0x6A
#define AFE_SLOTB_PD4_16BIT_REG         0x6B
#define AFE_A_PD1_LOW_REG               0x70
#define AFE_A_PD2_LOW_REG               0x71
#define AFE_A_PD3_LOW_REG               0x72
#define AFE_A_PD4_LOW_REG               0x73
#define AFE_A_PD1_HIGH_REG              0x74
#define AFE_A_PD2_HIGH_REG              0x75
#define AFE_A_PD3_HIGH_REG              0x76
#define AFE_A_PD4_HIGH_REG              0x77
#define AFE_B_PD1_LOW_REG               0x78
#define AFE_B_PD2_LOW_REG               0x79
#define AFE_B_PD3_LOW_REG               0x7A
#define AFE_B_PD4_LOW_REG               0x7B
#define AFE_B_PD1_HIGH_REG              0x7C
#define AFE_B_PD2_HIGH_REG              0x7D
#define AFE_B_PD3_HIGH_REG              0x7E
#define AFE_B_PD4_HIGH_REG              0x7F

// AFE configuration register power-up values for all R/W registers if
// register is commented out, the power-up configuration is the default
// AFE chip reset value. All register values are double-byte (16-bit).
//#define AFE_STATUS_PUP_VALUE                  0x0000  // address 0x00
#define AFE_INT_MASK_PUP_VALUE                0x01FF  // address 0x01
#define AFE_GPIO_DRV_PUP_VALUE                0x0005  // address 0x02
//#define AFE_FIFO_THRESH_PUP_VALUE             0x0000  // address 0x06
//#define AFE_DEVID_PUP_VALUE                   0x0516  // address 0x08
//#define AFE_I2C_ID_PUP_VALUE                  0x00C8  // address 0x09
//#define AFE_CLK_RATIO_PUP_VALUE               0x0000  // address 0x0A
#define AFE_GPIO_CTRL_PUP_VALUE               0x000D  // address 0x0B
//#define AFE_SLAVE_ADDRESS_KEY_PUP_VALUE       0x0000  // address 0x0D
//#define AFE_SW_RESET_PUP_VALUE                0x0000  // address 0x0F
#define AFE_MODE_STDBY_VALUE                  0x0000  // address 0x10
#define AFE_MODE_PRGM_VALUE                   0X0001  // address 0x10
#define AFE_MODE_NRM_VALUE                    0X0002  // address 0x10
#define AFE_SLOT_EN_PUP_VALUE                 0x1021  // address 0x11
#define AFE_FSAMPLE_PUP_VALUE                 0x01F4  // address 0x12
#define AFE_PD_LED_SELECT_PUP_VALUE           0x0559  // address 0x14
#define AFE_NUM_AVG_PUP_VALUE                 0x0550  // address 0x15
#define AFE_SLOTA_CH1_OFFSET_PUP_VALUE        0x1E78  // address 0x18
#define AFE_SLOTA_CH2_OFFSET_PUP_VALUE        0x1E78  // address 0x19
#define AFE_SLOTA_CH3_OFFSET_PUP_VALUE        0x1E78  // address 0x1A
#define AFE_SLOTA_CH4_OFFSET_PUP_VALUE        0x3FFF  // address 0x1B
#define AFE_SLOTB_CH1_OFFSET_PUP_VALUE        0x1E78  // address 0x1E
#define AFE_SLOTB_CH2_OFFSET_PUP_VALUE        0x1E78  // address 0x1F
#define AFE_SLOTB_CH3_OFFSET_PUP_VALUE        0x1E78  // address 0x20
#define AFE_SLOTB_CH4_OFFSET_PUP_VALUE        0x3FFF  // address 0x21
//#define AFE_ILED3_COARSE_PUP_VALUE            0x3000  // address 0x22
#define AFE_ILED1_COARSE_PUP_VALUE            0x300A  // address 0x23
#define AFE_ILED2_COARSE_PUP_VALUE            0x1004  // address 0x24
#define AFE_ILED_FINE_PUP_VALUE               0x628C  // address 0x25
#define AFE_SLOTA_LED_PULSE_PUP_VALUE         0x0319  // address 0x30
#define AFE_SLOTA_NUMPULSES_PUP_VALUE         0x0818  // address 0x31
//#define AFE_LED_DISABLE_PUP_VALUE             0x0000  // address 0x34
#define AFE_SLOTB_LED_PULSE_PUP_VALUE         0x0319  // address 0x35
#define AFE_SLOTB_NUMPULSES_PUP_VALUE         0x0818  // address 0x36
//#define AFE_ALT_PWR_DN_PUP_VALUE              0x0000  // address 0x37
//#define AFE_EXT_SYNC_STARTUP_PUP_VALUE        0x0000  // address 0x38
#define AFE_SLOTA_AFE_WINDOW_PUP_VALUE        0x21FE  // address 0x39
#define AFE_SLOTB_AFE_WINDOW_PUP_VALUE        0x21FE  // address 0x3B
//#define AFE_AFE_PWR_CFG1_PUP_VALUE            0x3006  // address 0x3C
#define AFE_SLOTA_TIA_CFG_PUP_VALUE           0x1C38  // address 0x42
//#define AFE_SLOTA_AFE_CFG_PUP_VALUE           0xADA5  // address 0x43
#define AFE_SLOTB_TIA_CFG_PUP_VALUE           0x1C38  // address 0x44
//#define AFE_SLOTB_AFE_CFG_PUP_VALUE           0xADA5  // address 0x45
#define AFE_SAMPLE_CLK_PUP_VALUE              0x26A2  // address 0x4B
//#define AFE_CLK32M_ADJUST_PUP_VALUE           0x0098  // address 0x4D
//#define AFE_ADC_CLOCK_PUP_VALUE               0x0060  // address 0x4E
//#define AFE_EXT_SYNC_SEL_PUP_VALUE            0x2090  // address 0x4F
//#define AFE_CLK_32M_CAL_EN_PUP_VALUE          0x0000  // address 0x50
//#define AFE_PWR_CFG2_PUP_VALUE                0x0020  // address 0x54
//#define AFE_TIA_INDEP_GAIN_PUP_VALUE          0x0000  // address 0x55
//#define AFE_DIGITAL_INT_EN_PUP_VALUE          0x0000  // address 0x58
//#define AFE_DIG_INT_CFG_PUP_VALUE             0x0000  // address 0x5A
#define AFE_DATA_ACCESS_CTL_PUP_VALUE         0x0000  // address 0x5F
#define AFE_DATA_ACCESS_CTL_HOLD_A_VALUE      0x0002  // address 0x5F
#define AFE_DATA_ACCESS_CTL_HOLD_B_VALUE      0x0004  // address 0x5F
#define AFE_DATA_ACCESS_CTL_HOLD_AB_VALUE     0x0006  // address 0x5F

const uint8_t TOTAL_NUMBER_REGISTERS = 51;

// variables to store default values on the GUI
const uint16_t YAXIS_LOWER_DEFAULT = 0;
const uint16_t YAXIS_UPPER_DEFAULT = 10;
const uint8_t TIME_RANGE_DEFAULT = 2; // time range value (in minutes)
const uint8_t RED_LED_COARSE_DEFAULT = 10;
const uint8_t NIR_LED_COARSE_DEFAULT = 4;
const uint8_t TIA_OPTION_DEFAULT = 0; // 200K gain default
const uint8_t SIGNAL_OFFSET_DEFAULT = 6800;
const uint8_t SAMPLE_AVG_DEFAULT = 1;
const uint8_t SAMPLE_DEFAULT = 16;
// END - DEFINE CONSTANTS************************************************


#include <Wire.h> // include I2C library
#include <avr/wdt.h> // include avr watchdog timer library
#include <MegunoLink.h> // include Megunolink default library 
#include <CommandHandler.h> // include serial comand handler library 


// BEGIN - MEGUNOLINK HANDLES********************************************
// add handles for photodiode 1 timeslot A and B respectively 

// adds a handle to send data to meguno time plots
TimePlot PD1TSA("PD1RED"), PD1TSB("PD1IR"),
         PD2TSA("PD2RED"), PD2TSB("PD2IR"),
         PD3TSA("PD3RED"), PD3TSB("PD3IR"),
         MarkedPoints("MarkedPoints");

// adds handle to send data to megunolink tables 
InterfacePanel MyPanel;

// adds a handle to update no graph items on interface panel  
Table MyTable;

// allows serial commands to be processed 
CommandHandler<20, 60, 10> SerialCommandHandler; 
// END - MEGUNOLINK HANDLES**********************************************


// BEGIN - GLOBAL VARIABLES**********************************************
// declare and initialize global variables

// array to store all registers 
byte Registers[] ={
    AFE_STATUS_REG,
    AFE_INT_MASK_REG,
    AFE_GPIO_DRV_REG,
    AFE_FIFO_THRESH_REG,
    AFE_DEVID_REG,
    AFE_I2C_ID_REG,
    AFE_CLK_RATIO_REG,
    AFE_GPIO_CTRL_REG,
    AFE_SLAVE_ADDRESS_KEY_REG,
    AFE_SW_RESET_REG,
    AFE_MODE_REG,
    AFE_SLOT_EN_REG,
    AFE_FSAMPLE_REG,
    AFE_PD_LED_SELECT_REG,
    AFE_NUM_AVG_REG,
    AFE_SLOTA_CH1_OFFSET_REG,
    AFE_SLOTA_CH2_OFFSET_REG,
    AFE_SLOTA_CH3_OFFSET_REG,
    AFE_SLOTA_CH4_OFFSET_REG,
    AFE_SLOTB_CH1_OFFSET_REG,
    AFE_SLOTB_CH2_OFFSET_REG,
    AFE_SLOTB_CH3_OFFSET_REG,
    AFE_SLOTB_CH4_OFFSET_REG,
    AFE_ILED3_COARSE_REG,
    AFE_ILED1_COARSE_REG,
    AFE_ILED2_COARSE_REG,
    AFE_ILED_FINE_REG,
    AFE_SLOTA_NUMPULSES_REG,
    AFE_LED_DISABLE_REG,
    AFE_SLOTB_LED_PULSE_REG,
    AFE_SLOTB_NUMPULSES_REG,
    AFE_ALT_PWR_DN_REG,
    AFE_EXT_SYNC_STARTUP_REG,
    AFE_SLOTA_AFE_WINDOW_REG,
    AFE_SLOTB_AFE_WINDOW_REG,
    AFE_AFE_PWR_CFG1_REG,
    AFE_SLOTA_TIA_CFG_REG,
    AFE_SLOTA_AFE_CFG_REG,
    AFE_SLOTB_TIA_CFG_REG,
    AFE_SLOTB_AFE_CFG_REG,
    AFE_SAMPLE_CLK_REG,
    AFE_CLK32M_ADJUST_REG,
    AFE_ADC_CLOCK_REG,
    AFE_EXT_SYNC_SEL_REG,
    AFE_CLK_32M_CAL_EN_REG,
    AFE_PWR_CFG2_REG,
    AFE_TIA_INDEP_GAIN_REG,
    AFE_DIGITAL_INT_EN_REG,
    AFE_DIG_INT_CFG_REG,
    AFE_DATA_ACCESS_CTL_REG,
    AFE_FIFO_ACCESS_REG
  };

bool afeModConfigFlag = false; // AFE modify configuration flag
bool afeReadDataFlag = false; // AFE read data registers flag
bool afeRequestInfoFlag = false; // AFE request register without editing flag

// single-byte (8-bit) AFE register to modify
byte mlRegMod = 0x00;

// double-byte (16-bit word) value to read into the AFE register
word mlValMod = 0x0000;

// 16 bit word bytes that will be modified 
word mlByteMod = 0x0000;

// variables to store photodiode data
float PD1R_calibrated = 0.0;
float PD2R_calibrated = 0.0;
float PD3R_calibrated = 0.0;
float PD1IR_calibrated = 0.0;
float PD2IR_calibrated = 0.0;
float PD3IR_calibrated = 0.0;

// calibration variables for each photodiode channel
// used by 'Cmd_Calibrate' and 'Cmd_SetCalibrateSetting'
uint16_t PD1R_calibration_val = 1;
uint16_t PD1IR_calibration_val = 1;
uint16_t PD2R_calibration_val = 1;
uint16_t PD2IR_calibration_val = 1;
uint16_t PD3R_calibration_val = 1;
uint16_t PD3IR_calibration_val = 1;

// variable to store the number of times "Toggle Autoscroll" button is pressed
// used by 'Cmd_ToggleAutoscroll'
uint8_t toggleAutoScrollCounter = 0;

// variable to store the number of times "Toggle Data Stream" button is pressed
// used by 'Cmd_ToggleDataStream'
uint8_t toggleDataStreamCounter = 0;
// END - GLOBAL VARIABLES************************************************


// BEGIN - USER-DEFINED FUNCTIONS****************************************

/**
 * @brief ISR for AFE read-to-read interrupt.
 * 
 */
void afeReadDataSet() {
  afeReadDataFlag = true; // set AFE read data flag
}

/**
 * @brief Function to write 2 bytes via I2C.
 * 
 * @param i2cWAdrs        I2C address to write
 * @param i2cWReg         AFE register to write to
 * @param i2cWVal         2-byte value of the write
 * 
 */
void i2cWrite2Bytes(byte i2cWAdrs, byte i2cWReg, word i2cWVal) {
  // declare array to hold 2 bytes for transfer to an AFE register
  byte writeBytes[2];
  // load array with 2 bytes
  writeBytes[0] = highByte(i2cWVal);  // HIGH byte
  writeBytes[1] = lowByte(i2cWVal);  // LOW byte
  // start talking to the AFE at the specified address
  Wire.beginTransmission(i2cWAdrs);
  // send register number to AFE
  Wire.write(i2cWReg);
  // write 2 bytes from array to AFE, write HIGH byte first
  Wire.write(writeBytes, 2);
  // complete transmission
  Wire.endTransmission();
}

/**
 * @brief Function to read 2 bytes via I2C.
 * 
 * @param i2cRAdrs        I2C address to read
 * @param i2cRReg         AFE register to read from
 * @return                word value from register
 * 
 */
word i2cRead2Bytes(byte i2cRAdrs, byte i2cRReg) {
  // declare and initialize variables
  byte readByteHigh = 0;
  byte readByteLow = 0;  
  word readWord = 0;
  
  // start talking to the AFE at the specified address
  Wire.beginTransmission(i2cRAdrs);
  // send register number to AFE
  Wire.write(i2cRReg);
  // complete transmission
  Wire.endTransmission();
  // request 2 bytes from AFE, read from HIGH byte first

  // FIXED Wire.h issue, compiler could not decide which requestFrom() func to use,
  // adding (byte) type conversions removes this ambiguity
  Wire.requestFrom( (byte) i2cRAdrs, (byte) 2);

  // wait for afe response
  while(Wire.available()) {
    // receive bytes
    readByteHigh = Wire.read();   // HIGH byte
    readByteLow = Wire.read();   // LOW byte
  }

  // combine the HIGH and LOW bytes into one 16-bit int
  // start by bitwise shifting 8-bit b1 by 8-bits into the HIGH byte of 16-bit val
  // complete by bitwise OR of the 8-bit b2 with the new 16-bit val
  return readWord = (readByteHigh << 8) | readByteLow;  
}

/**
 * @brief Function to enable the 32 kHz sample clock.
 * 
 */
void afeSmplClk() {
  // write to AFE - enable 32 kHz sample clock
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SAMPLE_CLK_REG, AFE_SAMPLE_CLK_PUP_VALUE);
  // read from AFE - check status of 32 kHz sample clock
  word smplClkStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SAMPLE_CLK_REG);

  // TODO: to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the 32 kHz sample clock status
  Serial.print("SAMPLE_CLK running when = 0x26A2: 0x"); 
  Serial.println(smplClkStatus, HEX);
  delay(0.5); // pause to display
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

/**
 * @brief Function to place the AFE in program mode.
 * 
 */
void afePrgmMode() {
  // write to AFE - place AFE in program mode
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG, AFE_MODE_PRGM_VALUE);
  // read from AFE - check status of program mode
  word prgStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG);

  // TODO: to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the AFE program mode status
  Serial.print("AFE in program mode when = 0x01: 0x"); 
  Serial.println(prgStatus, HEX);
  delay(0.5); // pause to display  
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

/**
 * @brief Function to load default start-up state into AFE registers. Writes to the AFE.
 * 
 */
void afeDfltSup() {
  // enable the gpio0 output pin on the AFE
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_GPIO_DRV_REG, AFE_GPIO_DRV_PUP_VALUE);

  // enable the ready-to-read AFE data register interrupt
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_GPIO_CTRL_REG, AFE_GPIO_CTRL_PUP_VALUE);
  
  // enable time slots A and B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOT_EN_REG, AFE_SLOT_EN_PUP_VALUE);

  // configure the sampling frequency
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_FSAMPLE_REG, AFE_FSAMPLE_PUP_VALUE);

  // configure the photodiodes connected during each time slot
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_PD_LED_SELECT_REG, AFE_PD_LED_SELECT_PUP_VALUE);

  // configure the number of sample averages during each time slot
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_NUM_AVG_REG, AFE_NUM_AVG_PUP_VALUE);

  // configure the offset for time slot A, channel 1, 2, and 3
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH1_OFFSET_REG, AFE_SLOTA_CH1_OFFSET_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH2_OFFSET_REG, AFE_SLOTA_CH2_OFFSET_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH3_OFFSET_REG, AFE_SLOTA_CH3_OFFSET_PUP_VALUE);

  // configure the offset for time slot B, channel 1, 2, and 3
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH1_OFFSET_REG, AFE_SLOTB_CH1_OFFSET_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH2_OFFSET_REG, AFE_SLOTB_CH2_OFFSET_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH3_OFFSET_REG, AFE_SLOTB_CH3_OFFSET_PUP_VALUE);

  // configure the LED pulse for time slot A and B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_LED_PULSE_REG, AFE_SLOTA_LED_PULSE_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_LED_PULSE_REG, AFE_SLOTB_LED_PULSE_PUP_VALUE);

  // configure the number of LED pulse for time slot A and B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_NUMPULSES_REG, AFE_SLOTA_NUMPULSES_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_NUMPULSES_REG, AFE_SLOTB_NUMPULSES_PUP_VALUE);

  // configure the AFE window for time slot A and B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_AFE_WINDOW_REG, AFE_SLOTA_AFE_WINDOW_PUP_VALUE);
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_AFE_WINDOW_REG, AFE_SLOTB_AFE_WINDOW_PUP_VALUE);

  // TODO: set additional registers for default setup%%%%%%%%%%%%%%%%%%%%%%%%%%
  // // configure current values of RED and NIR LEDs to max intensity (do not increase)
  // i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_ILED1_COARSE_REG, AFE_ILED1_COARSE_PUP_VALUE);
  // i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_ILED2_COARSE_REG, AFE_ILED2_COARSE_PUP_VALUE);
  // i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_ILED_FINE_REG, AFE_ILED_FINE_PUP_VALUE);
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

/**
 * @brief Function to place the AFE in normal mode.
 * 
 */
void afeNrmMode() {
  // write to AFE - place AFE in normal mode
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG, AFE_MODE_NRM_VALUE);
  // read from AFE - check status of normal mode
  word nrmStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG);

  // TODO: to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the AFE program mode status
  Serial.print("AFE in normal mode when = 0x02: 0x"); 
  Serial.println(nrmStatus, HEX);
  delay(0.5); // pause to display
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

/**
 * @brief Function to update AFE registers per MegunoLink command.
 * 
 * @param mlRgMd          AFE register number
 * @param mlBtMd          bits to modify
 * @param mlVlMd          value to modify
 * 
 */
void afeModConfig(byte mlRgMd, word mlBtMd, word mlVlMd) {
  // declare and initialize variables
  word prstRegVal = 0;
  word invMlBtMd = 0;
  word andPrvIbp = 0;
  word orBaoRuv = 0;

  // place the AFE in program mode prior to changing any register values
  afePrgmMode();

  // read present value of the register to modify
  Serial.print("mlVlMd ");
  Serial.println(mlVlMd);
  prstRegVal =  i2cRead2Bytes(AFE_I2C_ADDRESS, mlRgMd);
  Serial.print("prstRegVal ");
  Serial.println(prstRegVal);
  // invert the bit positions
  Serial.print("mlBtMd ");
  Serial.println(mlBtMd);
  invMlBtMd = ~mlBtMd;
  Serial.print("invMlBtMd ");
  Serial.println(invMlBtMd);
  // bitwise AND present register value and the inverted bit positions
  andPrvIbp = prstRegVal & invMlBtMd;
  Serial.print("andPrvIbp ");
  Serial.println(andPrvIbp);
  // bitwise OR the bitwise AND output and the register update value
  orBaoRuv = andPrvIbp | mlVlMd;
  Serial.print("orBaoRuv ");
  Serial.println(orBaoRuv);
  // write to AFE - updates to AFE config
  i2cWrite2Bytes(AFE_I2C_ADDRESS, mlRgMd, orBaoRuv);

  // place the AFE into normal mode after modifying the configuration
  afeNrmMode();  
}

/**
 * @brief Function to read AFE 16-bit data registers containing photodiode data.
 * 
 */
void afeReadData() {
  // hold time slots A&B data to prevent sample updates during register read
  i2cWrite2Bytes(AFE_I2C_ADDRESS,AFE_DATA_ACCESS_CTL_REG, AFE_DATA_ACCESS_CTL_HOLD_AB_VALUE);

  // read raw data from photodiode 1, 2, 3 and time slot A, B 16-bit data registers
  word afePd1TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD1_16BIT_REG);
  word afePd2TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD2_16BIT_REG);
  word afePd3TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD3_16BIT_REG); 
  word afePd1TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD1_16BIT_REG);
  word afePd2TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD2_16BIT_REG);
  word afePd3TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD3_16BIT_REG);

  // release time slots A&B data to allow sample updates to data registers
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_DATA_ACCESS_CTL_REG, AFE_DATA_ACCESS_CTL_PUP_VALUE);

  // checks if any inputed value causes a divide-by-zero error. if 0 is encountered, set calibration value to 1
  if ((PD1R_calibration_val == 0) || (PD2R_calibration_val == 0) || (PD3R_calibration_val == 0) ||
      (PD1IR_calibration_val == 0) || (PD2IR_calibration_val == 0) || (PD3IR_calibration_val == 0)) {
        SetCalibrationToDefault();
        Serial.println("\nERROR: Calibration values cannot be 0.");
  }
  
  // calibrate raw data and store in variable
  PD1R_calibrated = ((float)afePd1TSA16) / PD1R_calibration_val;
  PD2R_calibrated = ((float)afePd2TSA16) / PD2R_calibration_val;
  PD3R_calibrated = ((float)afePd3TSA16) / PD3R_calibration_val;
  PD1IR_calibrated = ((float)afePd1TSB16) / PD1IR_calibration_val;
  PD2IR_calibrated = ((float)afePd2TSB16) / PD2IR_calibration_val;
  PD3IR_calibrated = ((float)afePd3TSB16) / PD3IR_calibration_val;
  
  // sends calibrated data from photodiode 1, 2, 3 and timeslot A, B to GUI time plots
  PD1TSA.SendData(F("PD1RED"), PD1R_calibrated);
  PD2TSA.SendData(F("PD2RED"), PD2R_calibrated);
  PD3TSA.SendData(F("PD3RED"), PD3R_calibrated);
  PD1TSB.SendData(F("PD1IR"), PD1IR_calibrated);
  PD2TSB.SendData(F("PD2IR"), PD2IR_calibrated);
  PD3TSB.SendData(F("PD3IR"), PD3IR_calibrated);

  // set dynamic data labels to corresponding photodiode data in "Channel Data" on GUI
  MyPanel.SetText(F("PD1R_DataLabel"), PD1R_calibrated, 2);
  MyPanel.SetText(F("PD2R_DataLabel"), PD2R_calibrated, 2);
  MyPanel.SetText(F("PD3R_DataLabel"), PD3R_calibrated, 2);
  MyPanel.SetText(F("PD1IR_DataLabel"), PD1IR_calibrated, 2);
  MyPanel.SetText(F("PD2IR_DataLabel"), PD2IR_calibrated, 2);
  MyPanel.SetText(F("PD3IR_DataLabel"), PD3IR_calibrated, 2);
}

/**
 * @brief Function to receive data from interface panel
 * 
 * @param Parameters      MegunoLink passed-in parameters
 * 
 */
void Cmd_MlRequest(CommandParameter &Parameters){
  afeRequestInfoFlag = Parameters.NextParameterAsInteger ();
  mlRegMod = Parameters.NextParameterAsInteger ();
  mlByteMod = Parameters.NextParameterAsInteger ();
  mlValMod = Parameters.NextParameterAsInteger ();
}

/**
 * @brief Function that receives register and data values from GUI
 * 
 * @param Parameters      MegunoLink passed-in parameters
 * 
 */
void Cmd_MlReceive(CommandParameter &Parameters){
  afeModConfigFlag = Parameters.NextParameterAsInteger ();
  mlRegMod = Parameters.NextParameterAsInteger ();
  mlByteMod = Parameters.NextParameterAsInteger ();
  mlValMod = Parameters.NextParameterAsInteger ();

  Serial.print("mlRegMod at input");
  Serial.println(mlRegMod);
  Serial.print("mlByteMod at input");
  Serial.println(mlByteMod);
  Serial.print("mlValMod at input");
  Serial.println(mlValMod);
}

/**
 * @brief Function to send all register values to chart in megunolink
 * 
 */
void ReadAllRegisters(){
  int count = 0;
  while(count < TOTAL_NUMBER_REGISTERS){
    word value = i2cRead2Bytes(AFE_I2C_ADDRESS, Registers[count]);
    MyTable.SendData(Registers[count], value, "Register, Value (decimal)");
    count++;
  }
}

/**
 * @brief Command handler sets the value of global calibration variables to whatever is passed in.
 * 
 * Ex.
 *     '!Calibrate 1000 2000 3000 4000 5000 6000 \r\n'
 * Result:
 * PD1R_calibration_val -> 1000
 * PD2R_calibration_val -> 2000
 * PD3R_calibration_val -> 3000
 * PD1IR_calibration_val -> 4000
 * PD2IR_calibration_val -> 5000
 * PD3IR_calibration_val -> 6000
 * 
 * This cmd is used by the calibration buttons on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters
 * 
 */
void Cmd_Calibrate(CommandParameter &Parameters){
  PD1R_calibration_val = Parameters.NextParameterAsInteger();
  PD2R_calibration_val = Parameters.NextParameterAsInteger();  
  PD3R_calibration_val = Parameters.NextParameterAsInteger();
  PD1IR_calibration_val = Parameters.NextParameterAsInteger();
  PD2IR_calibration_val = Parameters.NextParameterAsInteger();
  PD3IR_calibration_val = Parameters.NextParameterAsInteger();

  Serial.print("PD1R_calibration_val: ");
  Serial.println(PD1R_calibration_val);
  Serial.print("PD2R_calibration_val: ");
  Serial.println(PD2R_calibration_val);
  Serial.print("PD3R_calibration_val: ");
  Serial.println(PD3R_calibration_val);
  Serial.print("PD1IR_calibration_val: ");
  Serial.println(PD1IR_calibration_val);
  Serial.print("PD2IR_calibration_val: ");
  Serial.println(PD2IR_calibration_val);
  Serial.print("PD3IR_calibration_val: ");
  Serial.println(PD3IR_calibration_val);
}

/**
 * @brief Command handler sets the value of global calibration variables to predefined settings.
 * 
 * Ex.
 *    '!SetCalibrationSetting 0 \r\n'
 * Result:
 *    Used in conjunction with the combo value list on the GUI, this selects 1 of 4 hard-coded calibration settings.
 * This cmd is used by the "Apply Calibration" button on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters
 * 
 */
void Cmd_SetCalibrationSetting(CommandParameter &Parameters){
  int CalibrationSelection = Parameters.NextParameterAsInteger();
  if (CalibrationSelection == 0) { // values from Mourad's experiment (200k gain)
    PD1R_calibration_val = 586;
    PD2R_calibration_val = 586;
    PD3R_calibration_val = 1236;
    PD1IR_calibration_val = 2420;
    PD2IR_calibration_val = 2363;
    PD3IR_calibration_val = 2051;
  } else if (CalibrationSelection == 1) { // unused
    PD1R_calibration_val = 1;
    PD2R_calibration_val = 1;
    PD3R_calibration_val = 1;
    PD1IR_calibration_val = 1;
    PD2IR_calibration_val = 1;
    PD3IR_calibration_val = 1;
  } else if (CalibrationSelection == 2) { // unused
    PD1R_calibration_val = 2;
    PD2R_calibration_val = 1;
    PD3R_calibration_val = 1;
    PD1IR_calibration_val = 1;
    PD2IR_calibration_val = 1;
    PD3IR_calibration_val = 1;
  } else if (CalibrationSelection == 3) { // unused
    PD1R_calibration_val = 3;
    PD2R_calibration_val = 1;
    PD3R_calibration_val = 1;
    PD1IR_calibration_val = 1;
    PD2IR_calibration_val = 1;
    PD3IR_calibration_val = 1;
  }
}

/**
 * @brief Command handler disables or enables the autoscrolling of all channels at once.
 * 
 * Ex.
 *    '!ToggleAutoscroll \r\n'
 * Result:
 *    All individual channel plots on the GUI will stop scrolling if previously on or
 * start scrolling if previously off.
 * 
 * This does not stop data from being sent to the device. Rather, it halts the display
 * of new data at user's will. 
 * 
 * This cmd is used by the "ToggleAutoscroll" button.
 * 
 */
void Cmd_ToggleAutoscroll(){
  if (toggleAutoScrollCounter % 2 == 0) { // if even, stop the scroll
    PD1TSA.Run(false);
    PD2TSA.Run(false);
    PD3TSA.Run(false);
    PD1TSB.Run(false);
    PD2TSB.Run(false);
    PD3TSB.Run(false);
  } else { // if odd, start the scroll
    PD1TSA.Run(true);
    PD2TSA.Run(true);
    PD3TSA.Run(true);
    PD1TSB.Run(true);
    PD2TSB.Run(true);
    PD3TSB.Run(true);
  }
  toggleAutoScrollCounter++; // keep track of number times function called
}

/**
 * @brief Command handler sends a visible data point at the current point in time, this point is saved
 * in the .csv file.
 * 
 * Ex.
 *    '!MarkPlotPoint \r\n'
 * Result:
 *    A dot can be seen in the time plots. The .csv file will contain the exact times when this command
 *    is used.
 * 
 * This cmd is used by "Calibrate" and "Remove Calibration" buttons.
 * 
 */
void Cmd_MarkPlotPoint(){
  MarkedPoints.SendData("MarkedPoints", 5, Plot::Cyan, Plot::Dotted);
}

/**
 * @brief Command handler sets the range of the x-axis by number of minutes.
 * 
 * Ex.
 *    '!Cmd_SetTimeRange 15 \r\n'
 * Result:
 *    Sets the x-axis range of all time plots to be 15 minutes in duration.
 * If the parameter is 0, the function will set the range to 30 seconds.
 * 
 * This cmd is used by the "Apply Range" button.
 * 
 * @param Parameters      MegunoLink passed-in parameters 
 * 
 */
void Cmd_SetTimeRange(CommandParameter &Parameters){
  int numMinutes = Parameters.NextParameterAsInteger();
  if (numMinutes == 0) { // if 0 minutes, set to 30 seconds
    PD1TSA.SetXRange(0.5 / 60);
    PD2TSA.SetXRange(0.5 / 60);
    PD3TSA.SetXRange(0.5 / 60);
    PD1TSB.SetXRange(0.5 / 60);
    PD2TSB.SetXRange(0.5 / 60);
    PD3TSB.SetXRange(0.5 / 60);
  } else { // if not 0 minutes, set to number of minutes given
    PD1TSA.SetXRange( ((float)numMinutes) / 60 );
    PD2TSA.SetXRange( ((float)numMinutes) / 60 );
    PD3TSA.SetXRange( ((float)numMinutes) / 60 );
    PD1TSB.SetXRange( ((float)numMinutes) / 60 );
    PD2TSB.SetXRange( ((float)numMinutes) / 60 );
    PD3TSB.SetXRange( ((float)numMinutes) / 60 );
  }
}

/**
 * @brief Command handler updates the y-axis scale for the individual time plots on the GUI.
 * 
 * Ex.
 *    '!SetYAxis 0 12000 0 0 0 1 1 1 \r\n'
 * Result:
 *    All NIR channel time plots on the GUI will begin at 0 and end at 12000.
 * 
 * This cmd is used by the "Set y-axis" button on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters  
 * 
 */
void Cmd_SetYAxis(CommandParameter &Parameters){
  // get command lower and upper bounds
  word YAxisLowerBound = Parameters.NextParameterAsInteger();
  word YAxisUpperBound = Parameters.NextParameterAsInteger();

  // get command booleans for checkboxes
  bool PD1RSelected = (bool)Parameters.NextParameterAsInteger();
  bool PD2RSelected = (bool)Parameters.NextParameterAsInteger();
  bool PD3RSelected = (bool)Parameters.NextParameterAsInteger();
  bool PD1NIRSelected = (bool)Parameters.NextParameterAsInteger();
  bool PD2NIRSelected = (bool)Parameters.NextParameterAsInteger();
  bool PD3NIRSelected = (bool)Parameters.NextParameterAsInteger();

  // update left and right y-axis scales for RED channel
  if (PD1RSelected) {
    PD1TSA.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD1TSA.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }
  if (PD2RSelected) {
    PD2TSA.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD2TSA.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }
  if (PD3RSelected) {
    PD3TSA.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD3TSA.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }
  // update left and right y-axis scales for NIR channel
  if (PD1NIRSelected) {
    PD1TSB.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD1TSB.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }
  if (PD2NIRSelected) {
    PD2TSB.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD2TSB.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }
  if (PD3NIRSelected) {
    PD3TSB.SetYRange(YAxisLowerBound, YAxisUpperBound);
    PD3TSB.SetY2Range(YAxisLowerBound, YAxisUpperBound);
  }

  Serial.print("YAxisLowerBound: ");
  Serial.println(YAxisLowerBound);
  Serial.print("YAxisUpperBound: ");
  Serial.println(YAxisUpperBound);
}

/**
 * @brief Command handler updates the status of the checkboxes in the "Plot Controls."
 * tab in MegunoLink.
 * 
 * Ex.
 *    '!SetYAxisCheckboxes 0 0 0 0 0 0 \r\n'
 * Result:
 *    All checkboxes are unchecked.
 * 
 * This cmd is used by the "Select All" and "Clear" buttons on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters   
 * 
 */
void Cmd_SetYAxisCheckboxes(CommandParameter &Parameters){
  MyPanel.SetCheck(F("PD1R_Checkbox"), (bool)Parameters.NextParameterAsInteger());
  MyPanel.SetCheck(F("PD2R_Checkbox"), (bool)Parameters.NextParameterAsInteger());
  MyPanel.SetCheck(F("PD3R_Checkbox"), (bool)Parameters.NextParameterAsInteger());
  MyPanel.SetCheck(F("PD1IR_Checkbox"), (bool)Parameters.NextParameterAsInteger());
  MyPanel.SetCheck(F("PD2IR_Checkbox"), (bool)Parameters.NextParameterAsInteger());
  MyPanel.SetCheck(F("PD3IR_Checkbox"), (bool)Parameters.NextParameterAsInteger());
}

/**
 * @brief Command handler sets the y-axis upper and lower bounds to given values.
 * 
 * Ex.
 *    '!SetYAxisUIValues 0 9000 \r\n'
 * Result:
 *    The GUI will display the upper and lower bounds as 9000 and 0, respectively.
 * 
 * This cmd is used by the "Set Y-axis to Default" and "Remove Calibration" buttons on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters    
 * 
 */
void Cmd_SetYAxisUIValues(CommandParameter &Parameters){
  int lower = Parameters.NextParameterAsInteger();
  int upper = Parameters.NextParameterAsInteger();
  MyPanel.SetNumber(F("YAxisLower"), lower);
  MyPanel.SetNumber(F("YAxisUpper"), upper);
  MyPanel.SetNumber(F("YAxisLowerDP"), lower);
  MyPanel.SetNumber(F("YAxisUpperDP"), upper);
}

/**
 * @brief Command handler sets bounds of x and y axes to their default values.
 * 
 * Ex.
 *    '!SetBoundsToDefault \r\n'
 * Result:
 *    All timeplot bounds will be reset to their default value (x = 2min range, y = 0-10)
 * 
 * This cmd used by the "Set Bounds to Default" button.
 * 
 */
void Cmd_SetBoundsToDefault(){
  Cmd_SetYAxisToDefault();
  Cmd_SetTimeRangeToDefault();
}

/**
 * @brief Command handler sets time plots upper, lower bounds to default.
 * 
 * Ex.
 *    '!SetYAxisToDefault \r\n'
 * Result:
 *    Time plot y-axes are set to default bounds.
 * 
 * This cmd is used by the "Set Y-axis to Default" button.
 * 
 */
void Cmd_SetYAxisToDefault(){
  // RED channel
  PD1TSA.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD1TSA.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD2TSA.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD2TSA.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD3TSA.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD3TSA.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  // NIR channel
  PD1TSB.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD1TSB.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD2TSB.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD2TSB.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD3TSB.SetYRange(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
  PD3TSB.SetY2Range(YAXIS_LOWER_DEFAULT, YAXIS_UPPER_DEFAULT);
}

/**
 * @brief Command handler sets time range to 2 minutes, updates GUI to reflect change.
 * 
 * Ex.
 *    '!SetTimeRangeToDefault \r\n'
 * Result:
 *    Time plots are set to 2 minute range on x-axis. "Time Range Options" valuebox reflects change.
 * 
 * This cmd is used by the "Set X-axis to Default" button.
 * 
 */
void Cmd_SetTimeRangeToDefault(){
  PD1TSA.SetXRange((float)TIME_RANGE_DEFAULT / 60); // 2 minutes (# min./# min. in hour)
  PD2TSA.SetXRange((float)TIME_RANGE_DEFAULT / 60);
  PD3TSA.SetXRange((float)TIME_RANGE_DEFAULT / 60);
  PD1TSB.SetXRange((float)TIME_RANGE_DEFAULT / 60);
  PD2TSB.SetXRange((float)TIME_RANGE_DEFAULT / 60);
  PD3TSB.SetXRange((float)TIME_RANGE_DEFAULT / 60);

  MyPanel.SetListValue(F("XAxisRangeOptions"), TIME_RANGE_DEFAULT);
  MyPanel.SetListValue(F("XAxisRangeOptionsDP"), TIME_RANGE_DEFAULT);
}

/**
 * @brief Command handler sets the GUI numbers to their default values.
 * 
 * Ex.
 *    '!SetControlsToDefault \r\n'
 * Result:
 *    All device settings will display their default values.
 * 
 * This cmd is used by the "Set to Default" button on the GUI.
 * 
 */
void Cmd_SetControlsToDefault(){
  MyPanel.SetNumber(F("RedLEDCoarse"), RED_LED_COARSE_DEFAULT);
  MyPanel.SetNumber(F("NIRLEDCoarse"), NIR_LED_COARSE_DEFAULT);
  MyPanel.SetListValue(F("TIAOptions"), TIA_OPTION_DEFAULT);
  MyPanel.SetNumber(F("OFFSET"), SIGNAL_OFFSET_DEFAULT);
  MyPanel.SetNumber(F("SampleAVG"), SAMPLE_AVG_DEFAULT);
  MyPanel.SetNumber(F("Sample"), SAMPLE_DEFAULT);
}

/**
 * @brief Command handler deletes all data points from all time plots on the GUI.
 * 
 * Ex.
 *    '!ClearData \r\n'
 * Result:
 *    All time plots are empty.
 * 
 * This cmd is used by the "Clear All Data" button on the GUI.
 * 
 */
void Cmd_ClearData(){
  PD1TSA.Clear();
  PD2TSA.Clear();
  PD3TSA.Clear();
  PD1TSB.Clear();
  PD2TSB.Clear();
  PD3TSB.Clear();
}

/**
 * @brief Command handler stops data from being sent to the GUI.
 * 
 * Ex.
 *    '!ToggleDataStream \r\n'
 * Result:
 *    All timeplots no longer update with new data.
 * 
 * This cmd is used by the "Toggle Data Stream" button on the GUI.
 * 
 */
void Cmd_ToggleDataStream(){
  if((toggleDataStreamCounter % 2) == 0) {
    afeModConfig(AFE_SAMPLE_CLK_REG, 0x80, 0); // disable sample clock
  } else {
    afeSmplClk(); // enable sample clock
  }
  toggleDataStreamCounter++; // track number of times function is called
}

/**
 * @brief Command handler zooms all time plots on the y-axis.
 * 
 * Ex.
 *    '!ZoomYAxis 5 \r\n'
 * Result:
 *    Time plots y-axis bounds are zoomed to +/- 5 units from latest data point.
 * 
 * This cmd is used by the "Zoom" button on the GUI.
 * 
 * @param Parameters      MegunoLink passed-in parameters       
 * 
 */
void Cmd_ZoomYAxis(CommandParameter &Parameters){
  int numChannels = 6;
  float zoomBounds = Parameters.NextParameterAsDouble();
  TimePlot plotArray[] = {PD1TSA, PD2TSA, PD3TSA, PD1TSB, PD2TSB, PD3TSB};
  float pdDataArray[] = {PD1R_calibrated, PD2R_calibrated, PD3R_calibrated,
    PD1IR_calibrated, PD2IR_calibrated, PD3IR_calibrated};

  for (int i = 0; i < numChannels; i++) {
    float lowerBound = pdDataArray[i] - zoomBounds;
    float upperBound = pdDataArray[i] + zoomBounds;

    // clip bounds if beyond register limits
    if (lowerBound < 0) {
      lowerBound = 0.0;
    }
    if (upperBound > 65535) {
      upperBound = 65535.0;
    }

    plotArray[i].SetYRange(lowerBound, upperBound);
    plotArray[i].SetY2Range(lowerBound, upperBound);
  }
}

// TODO: do we need this?
// /**
//  * @brief Helper function to allow code to modify AFE data registers.
//  * 
//  * @param modReg          AFE register number
//  * @param modByte         value of bits to modify
//  * @param modVal          value of overwrite bits
//  * 
//  */
// void ModifyRegister(byte modReg, word modByte, word modVal){
//   afeModConfigFlag = true;
//   mlRegMod = modReg;
//   mlByteMod = modByte;
//   mlValMod = modVal;

//   Serial.print("mlRegMod at input");
//   Serial.println(mlRegMod);
//   Serial.print("mlByteMod at input");
//   Serial.println(mlByteMod);
//   Serial.print("mlValMod at input");
//   Serial.println(mlValMod);
// }

/**
 * @brief Helper function invoked inside afeReadData, sets calibration to default values.
 * 
 */
void SetCalibrationToDefault(){
  PD1R_calibration_val = 1;
  PD2R_calibration_val = 1;
  PD3R_calibration_val = 1;
  PD1IR_calibration_val = 1;
  PD2IR_calibration_val = 1;
  PD3IR_calibration_val = 1;
}
// END - USER-DEFINED FUNCTIONS******************************************


void setup() {
  // enable watchdog timer with a 4 second timeout
  wdt_enable(WDTO_4S);

  // open the serial port to the PC at max speed
  Serial.begin(115200);

  // create a wire object
  Wire.begin();

  // enable the 32 kHz sample clock
  afeSmplClk();

  // place AFE in program mode from start-up standby state
  afePrgmMode();

  // load default start-up state into AFE registers
  afeDfltSup();
   
  // place AFE in normal mode
  afeNrmMode();

  // start AFE data register ready-to-read interrupt
  // AFE interrupt on arduino pin AFE_DATA_READY
  // ISR afeReadDataSet
  // interrupt triggered on FALLING edge (high to low) of input signal
  attachInterrupt(digitalPinToInterrupt(AFE_DATA_READY), afeReadDataSet, FALLING);

  // Adds "!MlReceive" function to receive data from Meguno via serial
  SerialCommandHandler.AddCommand(F("MlReceive"), Cmd_MlReceive);
  SerialCommandHandler.AddCommand(F("MlRequest"), Cmd_MlRequest);
  // Adds "!Calibrate" function to directly set calibration values
  SerialCommandHandler.AddCommand(F("Calibrate"), Cmd_Calibrate);
  // Adds "!SetCalibrationSetting" to allow the choice of using a pre-existing hard-coded calibration setting
  SerialCommandHandler.AddCommand(F("SetCalibrationSetting"), Cmd_SetCalibrationSetting);
  // Adds "!SetYAxis" function to change y-axis boundaries
  SerialCommandHandler.AddCommand(F("SetYAxis"), Cmd_SetYAxis);
  // Adds "!SetYAxisCheckboxes" function to set GUI checkboxes to a specific configuration.
  SerialCommandHandler.AddCommand(F("SetYAxisCheckboxes"), Cmd_SetYAxisCheckboxes);
  // Adds "!SetYAxisUIValues" function to set UI y-axis values to defined values.
  SerialCommandHandler.AddCommand(F("SetYAxisUIValues"), Cmd_SetYAxisUIValues);
  // Adds "!SetTimeRange" function to set the x-axis range in terms of minutes.
  SerialCommandHandler.AddCommand(F("SetTimeRange"), Cmd_SetTimeRange);
  // Adds "!SetTimeRangeToDefault" function to reset x-axis bounds to 2 minutes.
  SerialCommandHandler.AddCommand(F("SetTimeRangeToDefault"), Cmd_SetTimeRangeToDefault);
  // Adds "!SetYAxisToDefault" function to stop data from being sent to time plots.
  SerialCommandHandler.AddCommand(F("SetYAxisToDefault"), Cmd_SetYAxisToDefault);
  // Adds "!SetBoundsToDefault" function to set individual plots to their default x, y-axis bounds.
  SerialCommandHandler.AddCommand(F("SetBoundsToDefault"), Cmd_SetBoundsToDefault);
  // Adds "!SetControlsToDefault" function to set GUI controls to display default values.
  SerialCommandHandler.AddCommand(F("SetControlsToDefault"), Cmd_SetControlsToDefault);
  // Adds "!MarkPlotPoint" function to mark certain points in the data to aid in the identification of important data regions.
  SerialCommandHandler.AddCommand(F("MarkPlotPoint"), Cmd_MarkPlotPoint);
  // Adds "!ClearData" function to delete all data from time plots.
  SerialCommandHandler.AddCommand(F("ClearData"), Cmd_ClearData);
  // Adds "!ToggleAutoscroll" function to disable or enable autoscrolling.
  SerialCommandHandler.AddCommand(F("ToggleAutoscroll"), Cmd_ToggleAutoscroll);
  // Adds "!ToggleDataStream" function to stop data from being sent to time plots.
  SerialCommandHandler.AddCommand(F("ToggleDataStream"), Cmd_ToggleDataStream);
  // Adds "!ZoomYAxis" function to zoom time plots to a defined degree.
  SerialCommandHandler.AddCommand(F("ZoomYAxis"), Cmd_ZoomYAxis);

  // reset watchdog timer at end of setup
  wdt_reset();  
}


void loop() {
  // set AFE modify config flag
  // afeModConfigFlag = false;

  // single-byte (8-bit) AFE register to modify
  mlRegMod = 0x00;
  // double-byte (16-bit word) value to read into the AFE register
  mlValMod = 0x0000;
  // 16-bit word value of bytes within register to be changed 
  mlByteMod = 0x00;
  
  // processes serial comands from MegunoLink 
  SerialCommandHandler.Process();

  // AFE modify config flag is set when MegunoLink sends command
  if (afeModConfigFlag == true) {
    // update AFE configuration per MegunoLink command
    Serial.print("mlByteMod");
    Serial.println(mlByteMod);
    afeModConfig(mlRegMod, mlByteMod, mlValMod);
    // reset AFE modify config flag
    afeModConfigFlag = false;
  }
  
  // AFE read data flag is set when AFE data registers are ready to be read
  if (afeReadDataFlag == true) {
    // read AFE 16-bit data registers
    afeReadData();    
    // reset AFE read data flag
    afeReadDataFlag = false;
  }
  
  // AFE request info flag is set when user requests current value of a register without
  // changing it
  if(afeRequestInfoFlag==true){
    // if byte mod value is set to 1 all registers will be read and sent to chart regardsless
    // of register sent 
    if (mlByteMod==1){
      ReadAllRegisters();
    }
    //if byte mod value is set to anything other then 1 only a single register sent will be read
    else{
      long regvalue = i2cRead2Bytes(AFE_I2C_ADDRESS, mlRegMod);
    MyPanel.SetText(F("mlRegMod"), regvalue);
    Serial.println(regvalue);
    afeRequestInfoFlag = false;
    } 
  }

  // reset watchdog timer at end of loop
  wdt_reset();  
}
