/***************************************************************************************************/
/*
   This is a HAL based library for MCP4725, 12-bit Digital-to-Analog Converter with EEPROM

   NOTE:
   - operating/reference voltage 2.7v - 5.5v
   - add 100μF & 0.1 μF bypass capacitors within 4mm to Vdd
   - output voltage from 0 to operating voltage
   - maximum output current 25mA
   - output impedance 1 Ohm
   - maximum output load 1000pF/0.001μF in parallel with 5 kOhm
   - voltage settling time 6 μsec - 10 μsec 
   - slew rate 0.55 V/μs
   - device has 14-bit EEPROM with on-chip charge pump circuit for fail-safe writing
   - estimated EEPROM endurance 1 million write cycles
   - if Vdd < 2v all circuits & output disabled, when Vdd
     increases above Vpor device takes a reset state & upload data from EEPROM

   ported by : Salman Motlaq
	 sourse code: https://github.com/SMotlaq
	 
	 from an Arduino lib written by : enjoyneering79
   sourse code: https://github.com/enjoyneering

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "MCP4725.h"
#include "i2c.h"

/**************************************************************************/
/*
    MCP4725_init()

    Constructor
*/
/**************************************************************************/ 
MCP4725 MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725Ax_ADDRESS addr, float refV)
{
	MCP4725 _MCP4725;

	_MCP4725._i2cAddress = (uint16_t)(addr<<1);
	_MCP4725.hi2c = hi2c;

	MCP4725_setReferenceVoltage(&_MCP4725, refV); //set _refVoltage & _bitsPerVolt variables

	return _MCP4725;
}

/**************************************************************************/
/*
    MCP4725_isConnected()

    Check the connection 
*/
/**************************************************************************/ 
uint8_t MCP4725_isConnected(MCP4725* _MCP4725)
{
	return HAL_I2C_IsDeviceReady(_MCP4725->hi2c, _MCP4725->_i2cAddress, 2, 100) == HAL_OK;
}

/**************************************************************************/
/*
    setReferenceVoltage()

    Set reference voltage
*/
/**************************************************************************/
void MCP4725_setReferenceVoltage(MCP4725* _MCP4725, float value)
{
   if   (value == 0) _MCP4725->_refVoltage = MCP4725_REFERENCE_VOLTAGE; //sanity check, avoid division by zero
   else              _MCP4725->_refVoltage = value;    

   _MCP4725->_bitsPerVolt = (float)MCP4725_STEPS / _MCP4725->_refVoltage;         //TODO: check accuracy with +0.5
}

/**************************************************************************/
/*
    getReferenceVoltage()

    Return reference voltage
*/
/**************************************************************************/
float MCP4725_getReferenceVoltage(MCP4725* _MCP4725)
{
  return _MCP4725->_refVoltage;
}

/**************************************************************************/
/*
    setValue()

    Set output voltage to a fraction of Vref

    NOTE:
    -  mode:
      - "MCP4725_FAST_MODE"...........writes 2-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_REGISTER_MODE".......writes 3-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_EEPROM_MODE".........writes 3-bytes, data & power type to
                                      DAC register & EEPROM
    - powerType:
      - "MCP4725_POWER_DOWN_OFF"......power down off, draws 0.40mA no load
                                      & 0.29mA maximum load
      - "MCP4725_POWER_DOWN_1KOHM"....power down on with 1 kOhm to ground,
                                      draws 60nA
      - "MCP4725_POWER_DOWN_100KOHM"..power down on with 100 kOhm to ground
      - "MCP4725_POWER_DOWN_500KOHM"..power down on with 500kOhm to ground
*/
/**************************************************************************/ 
uint8_t MCP4725_setValue(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if (value > MCP4725_MAX_VALUE) value = MCP4725_MAX_VALUE; //make sure value never exceeds threshold
  #endif

  return MCP4725_writeComand(_MCP4725, value, mode, powerType);
}

/**************************************************************************/
/*
    setVoltage()

    Set output voltage to a fraction of Vref
*/
/**************************************************************************/ 
uint8_t MCP4725_setVoltage(MCP4725* _MCP4725, float voltage, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  uint16_t value = 0;

  /* convert voltage to DAC bits */
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if      (voltage >= _MCP4725->_refVoltage) value = MCP4725_MAX_VALUE;      					 //make sure value never exceeds threshold
  else if (voltage <= 0)					           value = 0;
  else                            					 value = voltage * _MCP4725->_bitsPerVolt; //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #else
  value = voltage * _MCP4725->_bitsPerVolt;                                											 //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #endif

  return MCP4725_writeComand(_MCP4725, value, mode, powerType);
}

/**************************************************************************/
/*
    getValue()

    Read current DAC value from DAC register

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725_getValue(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_DAC_REG); //D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx        

  if (value != MCP4725_ERROR) return value >> 4;       //00,00,00,00,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0
                              return value;            //collision on i2c bus
}

/**************************************************************************/
/*
    getVoltage()

    Read current DAC value from DAC register & convert to voltage
*/
/**************************************************************************/ 
float MCP4725_getVoltage(MCP4725* _MCP4725)
{
  float value = MCP4725_getValue(_MCP4725);

  if (value != MCP4725_ERROR) return value / _MCP4725->_bitsPerVolt;
                              return value;
}

/**************************************************************************/
/*
    getStoredValue()

    Read DAC value from EEPROM

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725_getStoredValue(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR) return value & 0x0FFF;  //00,00,00,00,D11,D10,D9,D8,   D7,D6,D5,D4,D3,D2,D1,D0
                              return value;           //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredVoltage()

    Read stored DAC value from EEPROM & convert to voltage
*/
/**************************************************************************/ 
float MCP4725_getStoredVoltage(MCP4725* _MCP4725)
{
  float value = MCP4725_getStoredValue(_MCP4725);

  if (value != MCP4725_ERROR) return value / _MCP4725->_bitsPerVolt;
                              return value;
}

/**************************************************************************/
/*
    getPowerType()

    Return current power type from DAC register

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - in the power-down modes Vout is off
    - see MCP4725 datasheet on p.15
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725_getPowerType(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR)
  {
           value &= 0x0006;                             //00,00,00,00,00,PD1,PD0,00
    return value >> 1;                                  //00,00,00,00,00,00,PD1,PD0
  }

  return value;                                         //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredPowerType()

    Return stored power type from EEPROM

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725_getStoredPowerType(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR)
  {
    value = value << 1;                               //PD1,PD0,xx,D11,D10,D9,D8,D7  D6,D5,D4,D3,D2,D1,D0,00
    return  value >> 14;                              //00,00,00,00,00,00,00,00      00,00,00,00,00,00,PD1,PD0
  }

  return value;                                       //collision on i2c bus
}

/**************************************************************************/
/*
    reset()

    Reset MCP4725 & upload data from EEPROM to DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - if Vdd < 2v all circuits & output disabled, when the Vdd
      increases above Vpor device takes a reset state
*/
/**************************************************************************/ 
void MCP4725_reset(MCP4725* _MCP4725)
{
  //Wire.beginTransmission(MCP4725_GENERAL_CALL_ADDRESS);
  //Wire.send(MCP4725_GENERAL_CALL_RESET);
  //Wire.endTransmission(true);
	
	uint8_t buffer[1] = {MCP4725_GENERAL_CALL_RESET};
	HAL_I2C_Master_Transmit(_MCP4725->hi2c, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 1000);
	
}

/**************************************************************************/
/*
    wakeUP()

    Wake up & upload value from DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - resets current power-down bits, EEPROM power-down bit are
      not affected
*/
/**************************************************************************/ 
void MCP4725_wakeUP(MCP4725* _MCP4725)
{
  //Wire.beginTransmission(MCP4725_GENERAL_CALL_ADDRESS);
  //Wire.send(MCP4725_GENERAL_WAKE_UP);
  //Wire.endTransmission(true);
	
	uint8_t buffer[1] = {MCP4725_GENERAL_WAKE_UP};
	HAL_I2C_Master_Transmit(_MCP4725->hi2c, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 1000);
	
}

/**************************************************************************/
/*
    getEepromBusyFlag()

    Return EEPROM writing status from DAC register 

    NOTE:
    - any new write command including repeat bytes during EEPROM write mode
      is ignored
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint8_t MCP4725_getEepromBusyFlag(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR) return (value & 0x80)==0x80;		//1 - completed, 0 - incompleted
                              return 0;										//collision on i2c bus
}

/**************************************************************************/
/*
    writeComand()

    Writes value to DAC register or EEPROM

    NOTE:
    - "MCP4725_FAST_MODE" bit format:
      15    14    13   12   11   10   9   8   7   6   5   4   3   2   1   0-bit
      C2=0, C1=0, PD1, PD0, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0
    - "MCP4725_REGISTER_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=0, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx
    - "MCP4725_EEPROM_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=1, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx

    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
*/
/**************************************************************************/ 
uint8_t	MCP4725_writeComand(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
	uint8_t buffer[3];
	HAL_StatusTypeDef I2C_Stat;
  //Wire.beginTransmission(_i2cAddress);

  switch (mode)
  {
    case MCP4725_FAST_MODE:                                            //see MCP4725 datasheet on p.18
		
      //Wire.send(mode | (powerType << 4)  | highByte(value));
      //Wire.send(lowByte(value));
		
			buffer[0] = mode | (powerType << 4)  | highByte(value);
			buffer[1] = lowByte(value);
		
			I2C_Stat = HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, 2, 1000);
		
      break;

    case MCP4725_REGISTER_MODE: case MCP4725_EEPROM_MODE:              //see MCP4725 datasheet on p.19
      value = value << 4;                                              //D11,D10,D9,D8,D7,D6,D5,D4,  D3,D2,D1,D0,xx,xx,xx,xx
      //Wire.send(mode  | (powerType << 1));
      //Wire.send(highByte(value));
      //Wire.send(lowByte(value));
      
			buffer[0] = mode  | (powerType << 1);
			buffer[1] = highByte(value);
			buffer[2] = lowByte(value);
		
			I2C_Stat = HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, 3, 1000);
		
			break;
  }

  if (I2C_Stat != HAL_OK) return 0;                   //send data over i2c & check for collision on i2c bus

  if (mode == MCP4725_EEPROM_MODE)
  {
    if (MCP4725_getEepromBusyFlag(_MCP4725) == 1) return 1;                      //write completed, success!!!
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME); //typical EEPROM write time 25 msec
    if (MCP4725_getEepromBusyFlag(_MCP4725) == 1) return 1;                      //write completed, success!!!
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME); //maximum EEPROM write time 25 + 25 = 50 msec
  }

  return 1;                                                         //success!!!
}

/**************************************************************************/
/*
    readRegister()

    Read DAC register via i2c bus

    NOTE:
    - read output bit format:
      39  38  37 36 35 34  33  32  31  30  29 28 27 26 25 24  23 22 21 20 19 18 17 16  15 14  13  12 11  10  9  8   7  6  5  4  3  2  1  0-bit
      BSY,POR,xx,xx,xx,PD1,PD0,xx, D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx, xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
      ------ Settings data ------  ---------------- DAC register data ---------------  ------------------- EEPROM data --------------------
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725_readRegister(MCP4725* _MCP4725, MCP4725_READ_TYPE dataType)
{
  uint16_t value = dataType;                             //convert enum to integer to avoid compiler warnings                                    
	uint16_t ret_val = 0 ;
	uint8_t buffer[dataType];
	HAL_StatusTypeDef I2C_Stat;
	
	I2C_Stat = HAL_I2C_Master_Receive(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, dataType, 1000);

  if (I2C_Stat != HAL_OK) return MCP4725_ERROR;


  /* read data from buffer */
  switch (dataType)
  {
    case MCP4725_READ_SETTINGS:
      ret_val = buffer[0];

      break;

    case MCP4725_READ_DAC_REG: case MCP4725_READ_EEPROM:

      ret_val = buffer[value-2];
      ret_val = (ret_val << 8) | buffer[value-1];
      break;
  }

  return ret_val;
}
