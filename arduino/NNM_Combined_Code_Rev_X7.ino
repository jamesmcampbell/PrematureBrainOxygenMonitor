/**
 * University of Washington - Bothell
 * NNM Capstone Project 2022-2023
 * 
 * @authors Adam Gibson, Joel Elliot
 * @version X7
 * 
 * Compatible with MegunoLink X5, X6, & X7
 * 
 * Purpose:
 * Provide a starting point for the combined code structure
 * (Arduino, MegunoLink, and AFE register configuration)
 * 
 * Hardware Setup:
 * Arduino Uno connected to an Analog Devices ADPD105 AFE
 * via an I2C converter and a 5V to 1.8V level converter for the GPIO line
 * 
 * Revised and modified by David J. Kim
 */

// begin - define constants**********************************
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

// AFE configuration register power-up values for all R/W registers
// if register is commented out, the power-up configuration is
// the default AFE chip reset value
// all register values are double-byte (16-bit)
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
#define AFE_FSAMPLE_PUP_VALUE                 0x0050  // address 0x12
#define AFE_PD_LED_SELECT_PUP_VALUE           0x0559  // address 0x14
#define AFE_NUM_AVG_PUP_VALUE                 0x0000  // address 0x15
#define AFE_SLOTA_CH1_OFFSET_PUP_VALUE        0x1F00  // address 0x18
#define AFE_SLOTA_CH2_OFFSET_PUP_VALUE        0x1F00  // address 0x19
#define AFE_SLOTA_CH3_OFFSET_PUP_VALUE        0x1F00  // address 0x1A
#define AFE_SLOTA_CH4_OFFSET_PUP_VALUE        0x3FFF  // address 0x1B
#define AFE_SLOTB_CH1_OFFSET_PUP_VALUE        0x1F00  // address 0x1E
#define AFE_SLOTB_CH2_OFFSET_PUP_VALUE        0x1F00  // address 0x1F
#define AFE_SLOTB_CH3_OFFSET_PUP_VALUE        0x1F00  // address 0x20
#define AFE_SLOTB_CH4_OFFSET_PUP_VALUE        0x3FFF  // address 0x21
//#define AFE_ILED3_COARSE_PUP_VALUE            0x3000  // address 0x22
#define AFE_ILED1_COARSE_PUP_VALUE            0x300A  // address 0x23
#define AFE_ILED2_COARSE_PUP_VALUE            0x300A  // address 0x24
//#define AFE_ILED_FINE_PUP_VALUE               0x630C  // address 0x25
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
//#define AFE_SLOTA_TIA_CFG_PUP_VALUE           0x1C38  // address 0x42
//#define AFE_SLOTA_AFE_CFG_PUP_VALUE           0xADA5  // address 0x43
//#define AFE_SLOTB_TIA_CFG_PUP_VALUE           0x1C38  // address 0x44
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
// end - define constants************************************

// begin - include libraries*********************************
// include I2C library
#include <Wire.h>
// include avr watchdog timer library
#include <avr/wdt.h>
// include Meunolink default library 
#include <MegunoLink.h>
// include serial comand handler library 
#include <CommandHandler.h>
//end - include libraries************************************

// begin Meguno handles
// Add handles for photodiode 1 timeslot A and B respectivly 
TimePlot PD1TSA("PD1RED"), PD1TSB("PD1IR"),PD2TSA("PD2RED"), PD2TSB("PD2IR"), PD3TSA("PD3RED"),
  PD3TSB("PD3IR"), DataMark("DataMark");
// Adds a handle to send data to meguno time plots
InterfacePanel MyPanel;
//Adds handle to send data to megunolink tables 
Table MyTable;
// Adds a handle to update no graph items on interface panel  
CommandHandler<> SerialCommandHandler; 
//Allows serial commands to be processed 
//End Meguno handles 

// begin - global variables**********************************
// declare and initialize global variables
//begin array to store all registers 
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
  //end array with registers stored 
bool afeModConfigFlag = false;   // AFE modify configuration flag
bool afeReadDataFlag = false;   // AFE read data registers flag
bool afeRequestInfoFlag = false;  //AFE request register without editing flag
  // single-byte (8-bit) AFE register to modify
  byte mlRegMod = 0x00;
  // double-byte (16-bit word) value to read into the AFE register
  word mlValMod = 0x0000;
  // 16 bit word bytes that will be modified 
  word mlByteMod = 0x0000;
// end - global variables************************************

// begin - user defined functions****************************
// ISR for AFE read-to-read interrupt
void afeReadDataSet() {
  // set AFE read data flag
  afeReadDataFlag = true;
} // end - ISR for AFE read-to-read interrupt

// write 2 bytes via I2C
// void i2cWrite2Bytes(address, register, value) {}
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
} // end - write 2 bytes via I2C

// read 2 bytes via I2C
// 2_bytes_returned i2cRead2Bytes(address, register) {}
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
  Wire.requestFrom(i2cRAdrs, 2);
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
} // end - read 2 bytes via I2C

// enable the 32 kHz sample clock
void afeSmplClk() {
  // write to AFE - enable 32 kHz sample clock
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SAMPLE_CLK_REG, AFE_SAMPLE_CLK_PUP_VALUE);
  // read from AFE - check status of 32 kHz sample clock
  word smplClkStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SAMPLE_CLK_REG);

  // to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the 32 kHz sample clock status
  Serial.print("SAMPLE_CLK running when = 0x26A2: 0x"); 
  Serial.println(smplClkStatus, HEX);
  delay(0.5); // pause to display
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
} // end - enable the 32 kHz sample clock

// place the AFE in program mode
void afePrgmMode() {
  // write to AFE - place AFE in program mode
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG, AFE_MODE_PRGM_VALUE);
  // read from AFE - check status of program mode
  word prgStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG);

  // to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the AFE program mode status
  Serial.print("AFE in program mode when = 0x01: 0x"); 
  Serial.println(prgStatus, HEX);
  delay(0.5); // pause to display  
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
} // end - place the AFE in program mode

// load default start-up state into AFE registers
void afeDfltSup() {
  // begin start-up configuration to AFE registers#############################
  // write to AFE - enable the gpio0 output pin on the AFE
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_GPIO_DRV_REG, AFE_GPIO_DRV_PUP_VALUE);

  // write to AFE - enable the ready-to-read AFE data register interrupt
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_GPIO_CTRL_REG, AFE_GPIO_CTRL_PUP_VALUE);
  
  // write to AFE - enable time slots A and B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOT_EN_REG, AFE_SLOT_EN_PUP_VALUE);

  // write to AFE - configure the sampling frequency
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_FSAMPLE_REG, AFE_FSAMPLE_PUP_VALUE);

  // write to AFE - configure the photodiodes connected during each time slot
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_PD_LED_SELECT_REG, AFE_PD_LED_SELECT_PUP_VALUE);

  // write to AFE - configure the number of sample averages during each time slot
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_NUM_AVG_REG, AFE_NUM_AVG_PUP_VALUE);

  // write to AFE - configure the offset for time slot A, channel 1
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH1_OFFSET_REG, AFE_SLOTA_CH1_OFFSET_PUP_VALUE);

  // write to AFE - configure the offset for time slot A, channel 2
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH2_OFFSET_REG, AFE_SLOTA_CH2_OFFSET_PUP_VALUE);

  // write to AFE - configure the offset for time slot A, channel 3
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_CH3_OFFSET_REG, AFE_SLOTA_CH3_OFFSET_PUP_VALUE);

  // write to AFE - configure the offset for time slot B, channel 1
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH1_OFFSET_REG, AFE_SLOTB_CH1_OFFSET_PUP_VALUE);

  // write to AFE - configure the offset for time slot B, channel 2
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH2_OFFSET_REG, AFE_SLOTB_CH2_OFFSET_PUP_VALUE);

  // write to AFE - configure the offset for time slot B, channel 3
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_CH3_OFFSET_REG, AFE_SLOTB_CH3_OFFSET_PUP_VALUE);

  // write to AFE - configure the LED pulse for time slot A
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_LED_PULSE_REG, AFE_SLOTA_LED_PULSE_PUP_VALUE);

  // write to AFE - configure the number of LED pulse for time slot A
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_NUMPULSES_REG, AFE_SLOTA_NUMPULSES_PUP_VALUE);

  // write to AFE - configure the LED pulse for time slot B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_LED_PULSE_REG, AFE_SLOTB_LED_PULSE_PUP_VALUE);

  // write to AFE - configure the number of LED pulse for time slot B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_NUMPULSES_REG, AFE_SLOTB_NUMPULSES_PUP_VALUE);

  // write to AFE - configure the AFE window for time slot A
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_AFE_WINDOW_REG, AFE_SLOTA_AFE_WINDOW_PUP_VALUE);
  
  // write to AFE - configure the AFE window for time slot B
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_AFE_WINDOW_REG, AFE_SLOTB_AFE_WINDOW_PUP_VALUE);
  // end start-up configuration to AFE registers##############################
} // end - load default start-up state into AFE registers

// place the AFE in normal mode
void afeNrmMode() {
  // write to AFE - place AFE in normal mode
  i2cWrite2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG, AFE_MODE_NRM_VALUE);
  // read from AFE - check status of normal mode
  word nrmStatus = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_MODE_REG);

  // to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the AFE program mode status
  Serial.print("AFE in normal mode when = 0x02: 0x"); 
  Serial.println(nrmStatus, HEX);
  delay(0.5); // pause to display
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
} // end - place the AFE in normal mode

// update AFE configuration per MegunoLink command
// register number, bits to modify, and value to modify
// void afeModConfig(register, bits, value) {}
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
} // end - update AFE configuration per MegunoLink command

// read AFE 16-bit data registers
void afeReadData() {
  // hold time slots A&B data to prevent sample updates during register read
  i2cWrite2Bytes(AFE_I2C_ADDRESS,AFE_DATA_ACCESS_CTL_REG, AFE_DATA_ACCESS_CTL_HOLD_AB_VALUE);
  
  // read data from photodiode 1, time slot A, 16-bit data registers
  word afePd1TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD1_16BIT_REG);
  word afePd2TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD2_16BIT_REG);
  word afePd3TSA16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTA_PD3_16BIT_REG); 
  // read data from photodiode 1, time slot B, 16-bit data registers
  word afePd1TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD1_16BIT_REG);
  word afePd2TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD2_16BIT_REG);
  word afePd3TSB16 = i2cRead2Bytes(AFE_I2C_ADDRESS, AFE_SLOTB_PD3_16BIT_REG);

  // release time slots A&B data to allow sample updates to data registers
  i2cWrite2Bytes(AFE_I2C_ADDRESS,AFE_DATA_ACCESS_CTL_REG, AFE_DATA_ACCESS_CTL_PUP_VALUE);
  
  // updated SendData calls to add PD2 & PD3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Begin Routing to MegunLink
  //Writes data from photodiode 1 timeslot A to graph channel PD1TSA
  PD1TSA.SendData(F("PD1RED"), afePd1TSA16);
  //Writes data from photodiode 2 timeslot A to graph channel PD2TSA
  PD2TSA.SendData(F("PD2RED"), afePd2TSA16);
  //Writes data from photodiode 3 timeslot A to graph channel PD3TSA
  PD3TSA.SendData(F("PD3RED"), afePd3TSA16);
  //Writes data from photodiode 1 timeslot B to graph channel PD1TSB
  PD1TSB.SendData(F("PD1IR"), afePd1TSB16);
  //Writes data from photodiode 2 timeslot B to graph channel PD2TSB
  PD2TSB.SendData(F("PD2IR"), afePd2TSB16);
  //Writes data from photodiode 3 timeslot B to graph channel PD3TSB
  PD3TSB.SendData(F("PD3IR"), afePd3TSB16);
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //End Routing to MegunoLink

  // to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // print the data from photodiode 1, time slot A, 16-bit data registers
  // Serial.print("SA-PD1-Red: 0x"); 
  // Serial.print(afePd1TSA16, HEX);
  // Serial.print("    SA-PD2-Red: 0x"); 
  // Serial.print(afePd2TSA16, HEX);
  // Serial.print("    SA-PD3-Red: 0x"); 
  // Serial.print(afePd3TSA16, HEX);
  // // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // // to be rerouted to MegunoLink%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // // print the data from photodiode 1, time slot B, 16-bit data registers
  // Serial.print("    SB-PD1-IR: 0x"); 
  // Serial.print(afePd1TSB16, HEX);
  // Serial.print("    SB-PD2-IR: 0x"); 
  // Serial.print(afePd2TSB16, HEX);
  // Serial.print("    SB-PD3-IR: 0x"); 
  // Serial.println(afePd3TSB16, HEX);
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
} // end - read AFE 16-bit data registers

// Begin MegunoLink function to recieve data from interface panel
// function that recieves register and data values from GUI
void Cmd_MlRequest( CommandParameter &Parameters){
  afeRequestInfoFlag = Parameters.NextParameterAsInteger ();
  mlRegMod = Parameters.NextParameterAsInteger ();
  mlByteMod = Parameters.NextParameterAsInteger ();
  mlValMod = Parameters.NextParameterAsInteger ();
}

// create function that recieves register and data values from GUI
void Cmd_MlReceive( CommandParameter &Parameters){
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
// End MegunoLink function to recieve data from interface panel 

//BEGIN function to send all register values to chart in megunolink
void ReadAllRegisters(){
  int count = 0;
  while(count<51){
    word value = i2cRead2Bytes(AFE_I2C_ADDRESS, Registers[count]);
    MyTable.SendData(Registers[count], value, "Register, Value (decimal)");
    count = count +1;
  }
}
//END function to send all register values to chart in megunolink

// end - user defined functions******************************

// begin - setup*********************************************
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

  // Adds "MlReceive" function to receive data from Meguno via serial
  SerialCommandHandler.AddCommand(F("MlReceive"), Cmd_MlReceive);

  SerialCommandHandler.AddCommand(F("MlRequest"), Cmd_MlRequest);


  //Adds all registers into array

  // reset watchdog timer at end of setup
  wdt_reset();  
}
// end - setup***********************************************

// begin - loop**********************************************
void loop() {
  // read in MegunoLink commands%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // set AFE modify config flag
  //afeModConfigFlag = false;
  // single-byte (8-bit) AFE register to modify
  mlRegMod = 0x00;
  // double-byte (16-bit word) value to read into the AFE register
  mlValMod = 0x0000;
  // 16-bit word value of bytes within register to be changed 
  mlByteMod = 0x00;
  
  //Processes serial comands from MegunoLink 
  SerialCommandHandler.Process();
  // AFE modify config flag is set when MegunoLink sends command
  if (afeModConfigFlag == true) {
    if (mlRegMod == 244){
      DataMark.SendData("DataMark", 100);
    }
    else{
    // update AFE configuration per MegunoLink command
      Serial.print("mlByteMod");
      Serial.println(mlByteMod);
      afeModConfig(mlRegMod, mlByteMod, mlValMod);
    }
    // reset AFE modify config flag
    afeModConfigFlag = false;
  } // end - AFE modify config
  
  // AFE read data flag is set when AFE data registers are ready to be read
  if (afeReadDataFlag == true) {
    // read AFE 16-bit data registers
    afeReadData();    
    // reset AFE read data flag
    afeReadDataFlag = false;
  } // end - AFE read data
  
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
// end - loop************************************************
