//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.c
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Sensor layer. Functions for sensor access
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "SHT2x.h"
#include "i2c.h"

//==============================================================================
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================
{
	uint8_t crc = 0;
	uint8_t byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ 0x131;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return 1;
  else return 0;
}

//===========================================================================
uint8_t SHT2x_ReadUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t buff[2];
	uint8_t checksum;   //variable for checksum byte
	uint8_t error=0;    //variable for error code

  error = HAL_I2C_Mem_Read(&hi2c1, I2C_ADR_R, USER_REG_R, 1, buff, 2, 1000 );
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W);
//  error |= I2c_WriteByte (USER_REG_R);
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);
//  *pRegisterValue = I2c_ReadByte(ACK);
  *pRegisterValue = buff[0];
//  =I2c_ReadByte(NO_ACK);
  checksum = buff[1];
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
//  I2c_StopCondition();
  return error;
}

//===========================================================================
uint8_t SHT2x_WriteUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t error=0;   //variable for error code

//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W);
//  error |= I2c_WriteByte (USER_REG_W);
//  error |= I2c_WriteByte (*pRegisterValue);
//  I2c_StopCondition();
  error = HAL_I2C_Mem_Write(&hi2c1, I2C_ADR_W, USER_REG_W, 1, pRegisterValue, 1, 1000);
  return error;
}

//===========================================================================
uint8_t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, uint16_t *pMeasurand)
//===========================================================================
{
	uint8_t  buff[3];
	uint8_t  checksum;   //checksum
	uint8_t  data[2];    //data array for checksum verification
	uint8_t  error=0;    //error variable
	uint16_t i;          //counting variable

  //-- write I2C sensor address and command --
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  switch(eSHT2xMeasureType)
  { case HUMIDITY:
	  	  	  	  	  //error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM);
	  	  	  	  	  error |= HAL_I2C_Mem_Read(&hi2c1, I2C_ADR_R, TRIG_RH_MEASUREMENT_HM, 1, buff, 3, 2000);
	  	  	  	  break;
    case TEMP    :
    				  //error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM);
    				  error |= HAL_I2C_Mem_Read(&hi2c1, I2C_ADR_R, TRIG_T_MEASUREMENT_HM, 1, buff, 3, 2000);
    			  break;
  }
//  //-- wait until hold master is released --
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);
//  SCL=HIGH;                     // set SCL I/O port as input
//  for(i=0; i<1000; i++)         // wait until master hold is released or
//  { DelayMicroSeconds(1000);    // a timeout (~1s) is reached
//    if (SCL_CONF==1) break;
//  }
//  //-- check for timeout --
//  if(SCL_CONF==0) error |= TIME_OUT_ERROR;

//  //-- read two data bytes and one checksum byte --
//  pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//  pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
  data[0] = buff[0];
  *pMeasurand = buff[0];
  *pMeasurand <<= 8;
  data[1] = buff[1];
  *pMeasurand |= buff[1];
//  checksum=I2c_ReadByte(NO_ACK);
  checksum=buff[2];

  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
//  I2c_StopCondition();
  return error;
}

//===========================================================================
uint8_t SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, uint16_t *pMeasurand)
//===========================================================================
{
	uint8_t  checksum;   //checksum
	uint8_t  data[2];    //data array for checksum verification
	uint8_t  error=0;    //error variable
	uint16_t i=0;        //counting variable

//  //-- write I2C sensor address and command --
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
//  switch(eSHT2xMeasureType)
//  { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_POLL); break;
//    case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_POLL);  break;
//    default: assert(0);
//  }
//  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
//  do
//  { I2c_StartCondition();
//    DelayMicroSeconds(10000);  //delay 10ms
//    if(i++ >= 20) break;
//  } while(I2c_WriteByte (I2C_ADR_R) == ACK_ERROR);
//  if (i>=20) error |= TIME_OUT_ERROR;
//
//  //-- read two data bytes and one checksum byte --
//  pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//  pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
//  checksum=I2c_ReadByte(NO_ACK);
//
//  //-- verify checksum --
//  error |= SHT2x_CheckCrc (data,2,checksum);
//  I2c_StopCondition();

  return error;
}

//===========================================================================
uint8_t SHT2x_SoftReset()
//===========================================================================
{
	uint8_t  error=0;           //error variable
	uint8_t data;

//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
//  error |= I2c_WriteByte (SOFT_RESET);                            // Command
//  I2c_StopCondition();
  HAL_I2C_Mem_Write(&hi2c1, I2C_ADR_W, SOFT_RESET, 1, &data, 1, 1000);

  HAL_Delay(16); // wait till sensor has restarted

  return error;
}

//==============================================================================
float SHT2x_CalcRH(uint16_t u16sRH)
//==============================================================================
{
	float humidityRH;              // variable for result

  u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --

  humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

//==============================================================================
float SHT2x_CalcTemperatureC(uint16_t u16sT)
//==============================================================================
{
	float temperatureC;            // variable for result

  u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

  //-- calculate temperature [°C] --
  temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}

//==============================================================================
uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[])
//==============================================================================
{
	uint8_t  error=0;                          //error variable

//  //Read from memory location 1
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
//  error |= I2c_WriteByte (0xFA);         //Command for readout on-chip memory
//  error |= I2c_WriteByte (0x0F);         //on-chip memory address
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
//  u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
//  I2c_ReadByte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
//  u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
//  I2c_ReadByte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
//  u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
//  I2c_ReadByte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
//  u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
//  I2c_ReadByte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
//  I2c_StopCondition();
//
//  //Read from memory location 2
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
//  error |= I2c_WriteByte (0xFC);         //Command for readout on-chip memory
//  error |= I2c_WriteByte (0xC9);         //on-chip memory address
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
//  u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
//  u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
//  I2c_ReadByte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
//  u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
//  u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
//  I2c_ReadByte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
//  I2c_StopCondition();

  return error;
}
