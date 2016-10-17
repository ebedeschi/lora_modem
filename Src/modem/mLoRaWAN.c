
/*
 *  Library for managing managing the LoRaWAN module
 * 
 *  Copyright (C) 2015 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
  
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:        1.2
 *  Design:         David Gascón
 *  Implementation: Luis Miguel Martí, Ruben Martin
 */
#ifndef __WPROGRAM_H__
#include "mUtil.h"
#endif

#include <string.h>
#include <stdlib.h>
#include "mLoRaWAN.h"

#include "stm32l4xx_hal.h"

/******************************************************************************
 * FLASH DEFINITIONS COMMANDS
 ******************************************************************************/
 const char command_00[] =   "sys reset\r\n";
 const char command_01[] =   "sys factoryRESET\r\n";
 const char command_02[] =   "sys get hweui\r\n";
 const char command_03[] =   "sys get vdd\r\n";
 const char command_04[] =   "mac reset %s\r\n";
 const char command_05[] =   "mac tx cnf %u %s\r\n";
 const char command_06[] =   "mac tx uncnf %u %s\r\n";
 const char command_07[] =   "mac join abp\r\n";
 const char command_08[] =   "mac save\r\n";
 const char command_09[] =   "mac pause\r\n";
 const char command_10[] =   "mac resume\r\n";
 const char command_11[] =   "mac set devaddr %s\r\n";
 const char command_12[] =   "mac set deveui %s\r\n";
 const char command_13[] =   "mac set appeui %s\r\n";
 const char command_14[] =   "mac set nwkskey %s\r\n";
 const char command_15[] =   "mac set appskey %s\r\n";
 const char command_16[] =   "mac set appkey %s\r\n";
 const char command_17[] =   "mac set pwridx %u\r\n";
 const char command_18[] =   "mac set dr %u\r\n";
 const char command_19[] =   "mac set adr %s\r\n";
 const char command_20[] =   "mac set ch freq %u %lu\r\n";
 const char command_21[] =   "mac set ch dcycle %u %u\r\n";
 const char command_22[] =   "mac set ch drrange %u %u %u\r\n";
 const char command_23[] =   "mac set ch status %u %s\r\n";
 const char command_24[] =   "mac get devaddr\r\n";
 const char command_25[] =   "mac get deveui\r\n";
 const char command_26[] =   "mac get appeui\r\n";
 const char command_27[] =   "mac get dr\r\n";
 const char command_28[] =   "mac get band\r\n";
 const char command_29[] =   "mac get pwridx\r\n";
 const char command_30[] =   "mac get adr\r\n";
 const char command_31[] =   "mac get dcycleps\r\n";
 const char command_32[] =   "mac get mrgn\r\n";
 const char command_33[] =   "mac get gwnb\r\n";
 const char command_34[] =   "mac get status\r\n";
 const char command_35[] =   "mac get ch freq %u\r\n";
 const char command_36[] =   "mac get ch dcycle %u\r\n";
 const char command_37[] =   "mac get ch drrange %u\r\n";
 const char command_38[] =   "mac get ch status %u\r\n";
 const char command_39[] =   "radio rx 0\r\n";
 const char command_40[] =   "radio tx %s\r\n";
 const char command_41[] =   "radio cw on\r\n";
 const char command_42[] =   "radio cw off\r\n";
 const char command_43[] =   "radio set mod %s\r\n";
 const char command_44[] =   "radio set freq %lu\r\n";
 const char command_45[] =   "radio set pwr %i\r\n";
 const char command_46[] =   "radio set sf %s\r\n";
 const char command_47[] =   "radio set rxbw %s\r\n";
 const char command_48[] =   "radio set rxbw %s.%s\r\n";
 const char command_49[] =   "radio set bitrate %u\r\n";
 const char command_50[] =   "radio set fdev %u\r\n";
 const char command_51[] =   "radio set prlen %u\r\n";
 const char command_52[] =   "radio set crc %s\r\n";
 const char command_53[] =   "radio set cr %s\r\n";
 const char command_54[] =   "radio set wdt %lu\r\n";
 const char command_55[] =   "radio set bw %u\r\n";
 const char command_56[] =   "radio get mod\r\n";
 const char command_57[] =   "radio get freq\r\n";
 const char command_58[] =   "radio get pwr\r\n";
 const char command_59[] =   "radio get sf\r\n";
 const char command_60[] =   "radio get rxbw\r\n";
 const char command_61[] =   "radio get bitrate\r\n";
 const char command_62[] =   "radio get cr\r\n";
 const char command_63[] =   "radio get wdt\r\n";
 const char command_64[] =   "radio get bw\r\n";
 const char command_65[] =   "radio get snr\r\n";
 const char command_66[] =   "radio get crc\r\n";
 const char command_67[] =   "radio get prlen\r\n";
 const char command_68[] =   "sys get ver\r\n";
 const char command_69[] =   "mac set retx %u\r\n";
 const char command_70[] =   "mac get retx\r\n";
 const char command_71[] =   "radio get fdev\r\n";
 const char command_72[] =   "mac set upctr %lu\r\n";
 const char command_73[] =   "mac get upctr\r\n";
 const char command_74[] =   "mac set dnctr %lu\r\n";
 const char command_75[] =   "mac get dnctr\r\n";
 const char command_76[] =   "mac join otaa\r\n";
 const char command_77[] =   "mac set linkchk %u\r\n";
 const char command_78[] =   "mac set rx2 %u %lu\r\n";
 const char command_79[] =   "mac set rxdelay1 %u\r\n";
 const char command_80[] =   "mac reset\r\n";


const char* const table_LoRaWAN_COMMANDS[] =
{   
    command_00, 
    command_01,
    command_02,
    command_03,
    command_04,
    command_05,
    command_06,
    command_07,
    command_08,
    command_09,
    command_10,
    command_11,
    command_12,
    command_13,
    command_14,
    command_15,
    command_16,
    command_17,
    command_18,
    command_19,
    command_20,
    command_21,
    command_22,
    command_23,
    command_24,
    command_25,
    command_26,
    command_27,
    command_28,
    command_29,
    command_30,
    command_31,
    command_32,
    command_33,
    command_34,
    command_35,
    command_36,
    command_37,
    command_38,
    command_39,
    command_40,
    command_41,
    command_42,
    command_43,
    command_44,
    command_45, 
    command_46,
    command_47,
    command_48,
    command_49,
    command_50,
    command_51,
    command_52,
    command_53,
    command_54,
    command_55,
    command_56,
    command_57,
    command_58,
    command_59,
    command_60,
    command_61,
    command_62,
    command_63,
    command_64,
    command_65,
    command_66,
    command_67,     
    command_68,
    command_69,
    command_70,
    command_71,
    command_72,
    command_73,
    command_74,
    command_75,
    command_76,
    command_77,
    command_78,
    command_79,
    command_80
};

/******************************************************************************
 * FLASH DEFINITIONS ANSWERS
 ******************************************************************************/
 const char answer_00[] =   "ok";
 const char answer_01[] =   "invalid_param";
 const char answer_02[] =   "no_free_ch";
 const char answer_03[] =   "mac_rx";
 const char answer_04[] =   "radio_tx_ok";
 const char answer_05[] =   "RN2483";
 const char answer_06[] =   "accepted";
 const char answer_07[] =   "radio_rx  ";
 const char answer_08[] =   "mac_tx_ok";
 const char answer_09[] =   "on";
 const char answer_10[] =   "off";
 const char answer_11[] =   "mac_paused";
 const char answer_12[] =   "radio_err";
 const char answer_13[] =   "\r\n";
 const char answer_14[] =   "4294967245";
 const char answer_15[] =   "mac_err";
 const char answer_16[] =   "invalid_data_len";
 const char answer_17[] =   "keys_not_init";
 const char answer_18[] =   "not_joined";
 const char answer_19[] =   "denied";
 const char answer_20[] =   "RN2903";


const char* const table_LoRaWAN_ANSWERS[] =
{   
    answer_00, 
    answer_01,
    answer_02,
    answer_03,
    answer_04,
    answer_05,
    answer_06,
    answer_07,
    answer_08,
    answer_09,
    answer_10,
    answer_11,
    answer_12,
    answer_13,
    answer_14,
    answer_15,
    answer_16,
    answer_17,
    answer_18,
    answer_19,
    answer_20
};


/******************************************************************************
 * User API
 ******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
// System functions
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief   This function powers on the module
 * 
 * @return  
 *  @arg    '0' if OK
 */
uint8_t ON()
{
	// P-MOS power gateing, 0 -> ON
    HAL_GPIO_WritePin(RFPOWER_GPIO_Port, RFPOWER_Pin, GPIO_PIN_RESET);
//    HAL_Delay(200);
    return LORAWAN_ANSWER_OK;
}

/*!
 * @brief   This function powers down the module
 * 
 * @return  
 *  @arg    '0' if OK
 */
uint8_t OFF()
{
	// P-MOS power gateing, 1 -> OFF
    HAL_GPIO_WritePin(RFPOWER_GPIO_Port, RFPOWER_Pin, GPIO_PIN_SET);
    return LORAWAN_ANSWER_OK;
}
 
/*!
 * @brief       This function resets and restart the stored internal configurations
 *              will be loaded upon reboot and saves modules version.
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '2' if no answer  
 * 
 */
uint8_t reset()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "sys reset" command
    sprintf(_command,table_LoRaWAN_COMMANDS[0]);
    // create "RN2483" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[5]);
    // create "RN2903" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[20]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,600);
    
    if (status == 1)
    {
        _version = RN2483_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        _version = RN2903_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else
    {
        _version = 0;
        return LORAWAN_NO_ANSWER;
    }
}

/*!
 * @brief       This function resets and restart by hardware pin
 *
 * @return
 *
 */
uint8_t resetHardware()
{
	HAL_GPIO_WritePin(RN_RESET_GPIO_Port, RN_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RN_RESET_GPIO_Port, RN_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	return LORAWAN_ANSWER_OK;
}


/*!
 * @brief   This function resets the module's configuration data and user
 *              EEPROM to factory default values, restarts the module and saves
 *              modules version.
 *
 * @return  
 *  @arg    '0' if OK
 *  @arg    '2' if no answer  
 * 
 */
uint8_t factoryReset()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "sys factoryRESET" command
    sprintf(_command,table_LoRaWAN_COMMANDS[1]);
    // create "RN2483" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[5]);
    // create "RN2903" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[20]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,5000);

    if (status == 1)
    {
        waitFor1_t("\r\n",1000);
        _version = RN2483_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        waitFor1_t("\r\n",1000);
        _version = RN2903_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets hardware EUI from the module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getEUI()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "sys get hweui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[2]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 0)
    {
        return LORAWAN_NO_ANSWER;
    }
    else if (status == 1)
    {       
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_eui,0x00,sizeof(_eui));
            strncpy(_eui, pch, sizeof(_eui));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}

/*!
 * @brief   This function gets hardware EUI from the module nad stores
 *          last 4 byte as devAddres
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getAddr()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "sys get hweui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[2]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 0)
    {
        return LORAWAN_NO_ANSWER;
    }
    else if (status == 1)
    {       
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_devAddr,0x00,sizeof(_devAddr));
            strncpy(_devAddr, pch+8, sizeof(_devAddr));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}


/*!
 * @brief   This functions gets supply power from the module
 *
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getSupplyPower()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "sys get vdd" command
    sprintf(_command,table_LoRaWAN_COMMANDS[3]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {   
        _supplyPower = parseIntValue();
        if (_supplyPower == 0)
        {           
            return LORAWAN_ANSWER_ERROR;
        }       

        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   Checks if module is ready to use and saves which kind of
 *              module has been plugged to Arduino, either RN2483 or RN2903
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *
 */
uint8_t check()
{   
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // create "sys get ver" command
    sprintf(_command,table_LoRaWAN_COMMANDS[68]);
    // create "RN2483" command
    sprintf(ans1,table_LoRaWAN_ANSWERS[5]);
    // create "RN2903" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[20]);
    // create "invalid_param" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,1000);

    if (status == 1)
    {
        _version = RN2483_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    if (status == 2)
    {
        _version = RN2903_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 3)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




////////////////////////////////////////////////////////////////////////////////
// LoRaWAN functions
////////////////////////////////////////////////////////////////////////////////



/*! 
 * @brief   This function is used to reset LoRaWAN configuration and set working band.
 * 
 * @param   char* band: working LoRaWAN band: "433","868" or "900"
 *
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t resetMacConfig(char* band)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    if ((strcmp(band, "433")) && (strcmp(band, "868")) && (strcmp(band, "900")))
    {
        return LORAWAN_INPUT_ERROR;
    }

    if (_version == RN2903_MODULE)
    {

        // create "mac reset" command
        sprintf(_command,table_LoRaWAN_COMMANDS[80]);
        // create "ok" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
        // create "invalid_param" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

        //send command and wait for ans
        status = sendCommand2_t(_command,ans1,ans2,1000);

        if (status == 1)
        {
            return LORAWAN_ANSWER_OK;
        }
        else if (status == 2)
        {
            return LORAWAN_ANSWER_ERROR;
        }
        else
        {
            return LORAWAN_NO_ANSWER;
        }

    }

    // create "mac reset" command
    sprintf(_command,table_LoRaWAN_COMMANDS[4], band);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,1000);

    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*! 
 * @brief   This function gets hardware EUI and sets MAC devEUI
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t setDeviceEUI_def()
{
    uint8_t error = getEUI();
    
    if (error != 0)
    {
        return error;
    }   
    
    return setDeviceEUI(_eui);
}


/*! 
 * @brief   This function sets  MAC devEUI
 * 
 * @param   char* EUI: EUI to be set
 * 
 * @remarks EUI is a sequence of digit representing the value of devEUI
 *          expressed in hexadecimal value (i.e. EUI = 0004A30B001A836D 
 *          – address is composed by the following byte stream: 
 *          0x00, 0x04, 0xA3, 0x0B, 0x00, 0x1A, 0x83, 0x6D - 16 digit 
 *          converted in 8 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setDeviceEUI(char* eui)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // check eui length
    if (strlen(eui)!=16) return LORAWAN_INPUT_ERROR;

    // check if eui is a hexadecimal string
    for (uint8_t i=0;i<16;i++)
    {
        if (((eui[i] < 0x30) || (eui[i] > 0x39)) && ((eui[i] < 0x41) || (eui[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set deveui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[12], eui);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);
    
    if (status == 1)
    {
        memset(_devEUI,0x00,sizeof(_devEUI));
        strncpy(_devEUI,eui,sizeof(_devEUI));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function gets the MAC device EUI from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getDeviceEUI()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get deveui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[25]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 0)
    {
        return LORAWAN_NO_ANSWER;
    }
    else if (status == 1)
    {       
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_devEUI,0x00,sizeof(_devEUI));
            strncpy(_devEUI, pch, sizeof(_devEUI));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}



/*! 
 * @brief   This function gets hardware EUI and sets MAC devAddress
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t setDeviceAddr_def()
{
    getAddr();
    return setDeviceAddr(_devAddr);
}



/*! 
 * @brief   This function sets  MAC devAddress
 * 
 * @param   char* addr: addr to be set
 * 
 * @remarks addr is a sequence of digit representing the value of addres
 *          expressed in hexadecimal value (i.e. addr = 001A836D – address
 *          is composed by the following byte stream: 0x00, 0x1A, 0x83, 0x6D 
 *          – 8 digit converted in 4 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setDeviceAddr(char* addr)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // check addr length
    if (strlen(addr)!=8) return LORAWAN_INPUT_ERROR;

    // check if addr is a hexadecimal string
    for (uint8_t i=0;i<8;i++)
    {
        if (((addr[i] < 0x30) || (addr[i] > 0x39)) && ((addr[i] < 0x41) || (addr[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set devaddr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[11], addr);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        memset(_devAddr,0x00,sizeof(_devAddr));
        strncpy(_devAddr,addr,sizeof(_devAddr));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the MAC device Address from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getDeviceAddr()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get devaddr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[24]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 0)
    {
        return LORAWAN_NO_ANSWER;
    }
    else if (status == 1)
    {       
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_devAddr,0x00,sizeof(_devAddr));
            strncpy(_devAddr, pch, sizeof(_devAddr));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}



/*! 
 * @brief   This function sets  MAC Network Session Key
 * 
 * @param   char* key: key to be set
 * 
 * @remarks key is a sequence of digit representing the value of NwkSKey
 *          expressed in hexadecimal value (i.e. key = 000102030405060708091011121314
 *          32 digit converted in 16 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setNwkSessionKey(char* key)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // check key length
    if (strlen(key)!=32) return LORAWAN_INPUT_ERROR;

    // check if key is a hexadecimal string
    for (uint8_t i=0;i<32;i++)
    {
        if (((key[i] < 0x30) || (key[i] > 0x39)) && ((key[i] < 0x41) || (key[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set nwkskey" command
    sprintf(_command,table_LoRaWAN_COMMANDS[14], key);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        memset(_nwkSKey,0x00,sizeof(_nwkSKey));
        strncpy(_nwkSKey,key,sizeof(_nwkSKey));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function sets  MAC appEUI
 * 
 * @param   char* EUI: EUI to be set
 * 
 * @remarks EUI is a sequence of digit representing the value of appEUI
 *          expressed in hexadecimal value (i.e.: EUI = 0001020304050607
 *          16 digit converted in 8 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setAppEUI(char* eui)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // check eui length
    if (strlen(eui)!=16) return LORAWAN_INPUT_ERROR;

    //check if eui is a hexadecimal string
    for (uint8_t i=0;i<16;i++)
    {
        if (((eui[i] < 0x30) || (eui[i] > 0x39)) && ((eui[i] < 0x41) || (eui[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set appeui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[13], eui);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        memset(_appEUI,0x00,sizeof(_appEUI));
        strncpy(_appEUI,eui,sizeof(_appEUI));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the MAC AppEUI from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getAppEUI()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get appeui" command
    sprintf(_command,table_LoRaWAN_COMMANDS[26]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 0)
    {
        return LORAWAN_NO_ANSWER;
    }
    else if(status == 1)
    {       
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_appEUI,0x00,sizeof(_appEUI));
            strncpy(_appEUI, pch, sizeof(_appEUI));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}


/*! 
 * @brief   This function sets  MAC App Key
 * 
 * @param   char* key: key to be set
 * 
 * @remarks key is a sequence of digit representing the value of AppKey
 *          expressed in hexadecimal value (i.e.: key = 000102030405060708091011121314
 *          32 digit converted in 16 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setAppKey(char* key)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // check key length
    if (strlen(key)!=32) return LORAWAN_INPUT_ERROR;

    //check if key is a hexadecimal string
    for (uint8_t i=0;i<32;i++)
    {
        if (((key[i] < 0x30) || (key[i] > 0x39)) && ((key[i] < 0x41) || (key[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set appkey" command
    sprintf(_command,table_LoRaWAN_COMMANDS[16], key);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        memset(_appKey,0x00,sizeof(_appKey));
        strncpy(_appKey,key,sizeof(_appKey));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*! 
 * @brief   This function sets  MAC App Session Key
 * 
 * @param   char* key: key to be set
 * 
 * @remarks key is a sequence of digit representing the value of AppSKey
 *          expressed in hexadecimal value (i.e.: key = 000102030405060708091011121314
 *          32 digit converted in 16 bytes).
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setAppSessionKey(char* key)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // check key length
    if (strlen(key)!=32) return LORAWAN_INPUT_ERROR;

    // check if key is a hexadecimal string
    for (uint8_t i=0;i<32;i++)
    {
        if (((key[i] < 0x30) || (key[i] > 0x39)) && ((key[i] < 0x41) || (key[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac set appskey" command
    sprintf(_command,table_LoRaWAN_COMMANDS[15], key);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        memset(_appSKey,0x00,sizeof(_appSKey));
        strncpy(_appSKey,key,sizeof(_appSKey));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function is used to configure the LoRaWAN RF power level
 *
 * @param   uint8_t index: power level to be set [0..5] for 433 MHz,
 *          [1..5] for 868 MHz and [5..10] for 900
 *  RN2483
 *  @arg    0 -> 20 dBm (if supported)
 *  @arg    1 -> 14 dBm
 *  @arg    2 -> 11 dBm
 *  @arg    3 -> 8 dBm
 *  @arg    4 -> 5 dBm
 *  @arg    5 -> 2 dBm
 *
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' if unrecognized module
 */
uint8_t setPower(uint8_t index)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    switch (_version)
    {
        case RN2483_MODULE:
                getBand();

                switch (atol(_band))
                {
                    case 868:
                            if ((index > 5) || (index < 1)) return LORAWAN_INPUT_ERROR;
                            else break;
                    case 400:
                            if ((index > 5) || (index < 0)) return LORAWAN_INPUT_ERROR;
                            else break;
                    default:
                            return LORAWAN_VERSION_ERROR;
                }
                break;

        case RN2903_MODULE:
                if ( index > 10 || index < 5 || index == 6) return LORAWAN_INPUT_ERROR;
                else break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac set pwrindx" command
    sprintf(_command,table_LoRaWAN_COMMANDS[17], index);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        _powerIndex = index;
        return LORAWAN_ANSWER_OK;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*! 
 * @brief   This function is used to read the power index from module
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getPower()
{
    uint8_t status; 
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get pwrindx" command
    sprintf(_command,table_LoRaWAN_COMMANDS[29]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);

    if (status == 1)
    {
        _powerIndex = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*! 
 * @brief   This function is used to configure the LoRaWAN RF data rate
 * 
 * @param   uint8_t index: data rate to be set [0..7] (RN2483)
 *  @arg    0 -> Lora: SF 12 / 125 kHz      Bit/s: 250
 *  @arg    1 -> Lora: SF 11 / 125 kHz      Bit/s: 440
 *  @arg    2 -> Lora: SF 10 / 125 kHz      Bit/s: 980
 *  @arg    3 -> Lora: SF 9 / 125 kHz       Bit/s: 1760
 *  @arg    4 -> Lora: SF 8 / 125 kHz       Bit/s: 3125
 *  @arg    5 -> Lora: SF 7 / 125 kHz       Bit/s: 5470
 *  @arg    6 -> Lora: SF 7 / 250 kHz       Bit/s: 11000
 *  @arg    7 ->    FSK: 50kbps
 *
 *          uint8_t index: data rate to be set [0..4] (RN2903)
 *  @arg    0 -> Lora: SF 10 / 125 kHz      Bit/s: 980
 *  @arg    1 -> Lora: SF 9 / 125 kHz       Bit/s: 1760
 *  @arg    2 -> Lora: SF 8 / 125 kHz       Bit/s: 3125
 *  @arg    3 -> Lora: SF 7 / 125 kHz       Bit/s: 5470
 *  @arg    4 -> Lora: SF 8 / 500 kHz       Bit/s: 12500
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' if unrecognized module
 */
uint8_t setDataRate(uint8_t datarate)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    switch (_version)
    {
        case RN2483_MODULE:
                if (datarate > 7) return LORAWAN_INPUT_ERROR;
                break;

        case RN2903_MODULE:
                if (datarate > 4) return LORAWAN_INPUT_ERROR;
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac set dr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[18], datarate);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);

    if (status == 1)
    {
        _dataRate = datarate;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function is used to read the data rate from module
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getDataRate()
{
    uint8_t status; 
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get dr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[27]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);

    if (status == 1)
    {
        _dataRate = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function saves config set into module's EEPROM
 * 
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t saveConfig()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac save" command
    sprintf(_command,table_LoRaWAN_COMMANDS[8]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,4000);
    
    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*! 
 * @brief   This function joins module to a network
 * 
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '3' if keys were not initiated
 */
uint8_t joinABP()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // create "mac join abp" command
    sprintf(_command,table_LoRaWAN_COMMANDS[7]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    // create "keys_not_init" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[17]);

    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);

    if (status == 1)
    {
        memset(ans1,0x00,sizeof(ans1));
        // create "accepted" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[6]);

        //wait for response
        if (waitFor1_t(ans1,800) == 1)
        {
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else if (status == 3)
    {
        return LORAWAN_INIT_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function joins module to a network
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '3' if keys were not initiated
 */
uint8_t joinOTAA()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // create "mac join otaa" command
    sprintf(_command,table_LoRaWAN_COMMANDS[76]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    // create "keys_not_init" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[17]);

    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);

    if (status == 1)
    {
        memset(ans1,0x00,sizeof(ans1));
        // create "accepted" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[6]);
        memset(ans2,0x00,sizeof(ans2));
        // create "denied" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[19]);

        //wait for response
        if (waitFor2_t(ans1,ans2,20000) == 1)
        {
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else if (status == 3)
    {
        return LORAWAN_INIT_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 *
 * @brief   This function sends a LoRaWAN packet and waits for ACK
 *
 * @param   char* data: data to be sent
 *          uint8_t port: port number to send data
 *
 * @remarks data is a sequence of digit representing the value of byte stream
 *          expressed in hexadecimal value (i.e.: payload =12A435 – the payload
 *          is composed by the following byte stream: 0x12, 0xA4, 0x35 – 6 digit
 *          converted in 3 bytes). The maximum length of frame is 584 digit (292 Bytes).
 *          User can check _datareceived to know if a downlink was performed
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '4' if data length error
 *  @arg    '5' if error when sending data
 *  @arg    '6' if module hasn't joined to a network
 *  @arg    '7' if input port parameter error
 */
uint8_t sendConfirmed(uint8_t port, char* payload)
{
    uint8_t status;
    char ans1[20];
    char ans2[20];
    char ans3[20];
    char ans4[20];

    // clear data received flag
    _dataReceived = false;

    // clear buffers
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // check port
    if (port > 223) return LORAWAN_INPUT_ERROR;

    // check if payload is a hexadecimal string
    for (uint8_t i=0;i<strlen(payload);i++)
    {
        if (((payload[i] < 0x30) || (payload[i] > 0x39)) && ((payload[i] < 0x41) || (payload[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac tx cnf <port> <data>" command
    sprintf(_command,table_LoRaWAN_COMMANDS[5],port,payload);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    // create "not_joined" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[18]);

    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,1000);

    if (status == 1)
    {
        // clear buffer
        memset(ans1,0x00,sizeof(ans1));
        memset(ans2,0x00,sizeof(ans2));
        memset(ans3,0x00,sizeof(ans3));
        memset(ans4,0x00,sizeof(ans4));

        // mac_rx
        sprintf(ans1,table_LoRaWAN_ANSWERS[3]);
        // mac_tx_ok
        strcpy(ans2,table_LoRaWAN_ANSWERS[8]);
        // mac_err
        strcpy(ans3,table_LoRaWAN_ANSWERS[15]);
        // invalid_data_len
        strcpy(ans4,table_LoRaWAN_ANSWERS[16]);

        //wait for response
        status = waitFor4_t(ans1, ans2, ans3, ans4, 180000);

        if (status == 1)
        {
            waitFor1_t("\r\n",500);
            if (_length > 0)
            {
                char* pch = strtok((char*) _buffer," \r\n");
                _port = atoi(pch);

                pch = strtok(NULL," \r\n");

                memset(_data,0x00,sizeof(_data));
                strncpy(_data, pch, sizeof(_data)-1);

                saveConfig();
                _dataReceived = true;
                return LORAWAN_ANSWER_OK;
            }
            else
            {
                saveConfig();
                return LORAWAN_ANSWER_OK;
            }
        }
        else if (status == 2)
        {
            saveConfig();
            return LORAWAN_ANSWER_OK;
        }
        else if (status == 3)
        {
            saveConfig();
            return LORAWAN_SENDING_ERROR;
        }
        else if (status == 4)
        {
            saveConfig();
            return LORAWAN_LENGTH_ERROR;
        }
        else
        {
            saveConfig();
            return LORAWAN_NO_ANSWER;
        }
    }
    else if (status == 2)
    {
        saveConfig();
        return LORAWAN_ANSWER_ERROR;
    }
    else if (status == 3)
    {
        saveConfig();
        return LORAWAN_NOT_JOINED;
    }
    else
    {
        saveConfig();
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * 
 * @brief   This function sends a LoRaWAN packet without ACK
 *  
 * @param   char* data: data to be sent
 *          uint8_t port: port number to send data
 * 
 * @remarks data is a sequence of digit representing the value of byte stream
 *          expressed in hexadecimal value (i.e.: payload =12A435 – the payload 
 *          is composed by the following byte stream: 0x12, 0xA4, 0x35 – 6 digit
 *          converted in 3 bytes). The maximum length of frame is 584 digit (292 Bytes).
 *          User can check _datareceived to know if a downlink was performed
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '4' if data length error
 *  @arg    '5' if error when sending data
 *  @arg    '6' if module hasn't joined to a network
 *  @arg    '7' if input port parameter error
 */
uint8_t sendUnconfirmed(uint8_t port, char* payload)
{
    uint8_t status;
    char ans1[20];
    char ans2[20];
    char ans3[20];
    char ans4[20];

    // clear data received flag
    _dataReceived = false;

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // check port
    if (port > 223) return LORAWAN_INPUT_ERROR;

    // check if payload is a hexadecimal string
    for (uint8_t i=0;i<strlen(payload);i++)
    {
        if (((payload[i] < 0x30) || (payload[i] > 0x39)) && ((payload[i] < 0x41) || (payload[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    // create "mac tx uncnf <port> <data>" command
    sprintf(_command,table_LoRaWAN_COMMANDS[6],port,payload);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    // create "not_joined" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[18]);

    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);

    if (status == 1)
    {
        // clear buffer
        memset(ans1,0x00,sizeof(ans1));
        memset(ans2,0x00,sizeof(ans2));
        memset(ans3,0x00,sizeof(ans3));
        memset(ans4,0x00,sizeof(ans4));
            
        // mac_rx <port>
        sprintf(ans1,table_LoRaWAN_ANSWERS[3]);
        // mac_tx_ok
        sprintf(ans2,table_LoRaWAN_ANSWERS[8]);
        // mac_err
        sprintf(ans3,table_LoRaWAN_ANSWERS[15]);
        // invalid_data_len
        sprintf(ans4,table_LoRaWAN_ANSWERS[16]);
        
        //wait for response
        status = waitFor4_t(ans1, ans2, ans3, ans4, 20000);
        
        if (status == 1)
        {
            waitFor1_t("\r\n",500);

            if (_length > 0)
            {
                char* pch = strtok((char*) _buffer," \r\n");
                _port = atoi(pch);

                pch = strtok(NULL," \r\n");

                memset(_data,0x00,sizeof(_data));
                strncpy(_data, pch, sizeof(_data)-1);

                saveConfig();
                _dataReceived = true;
                return LORAWAN_ANSWER_OK;
            }
            else
            {
                saveConfig();
                return LORAWAN_ANSWER_OK;
            }
        }
        else if (status == 2)
        {
            saveConfig();
            return LORAWAN_ANSWER_OK;
        }
        else if (status == 3)
        {
            saveConfig();
            return LORAWAN_SENDING_ERROR;
        }
        else if (status == 4)
        {
            saveConfig();
            return LORAWAN_LENGTH_ERROR;
        }
        else
        {
            saveConfig();
            return LORAWAN_NO_ANSWER;
        }
    }
    else if (status == 2)
    {
        saveConfig();
        return LORAWAN_ANSWER_ERROR;
    }
    else if (status == 3)
    {
        saveConfig();
        return LORAWAN_NOT_JOINED;
    }
    else
    {
        saveConfig();
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function is used to set the ADR status from module
 * 
 * @param   char* state: "on"/"off"
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *      @arg    '7' if input parameter error
 */
uint8_t setADR(char* state)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    // check state
    if ((strcmp(state, "on")) && (strcmp(state, "off"))) return LORAWAN_INPUT_ERROR;

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac set adr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[19],state);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
        
    if (status == 1)
    {
        if (strcmp(state, "on")  == 0) _adr = true;
        if (strcmp(state, "off") == 0) _adr = false;        
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function is used to read the ADR status from module
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getADR()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));
    
    // create "mac get adr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[30]);
    sprintf(ans1,table_LoRaWAN_ANSWERS[9]);
    sprintf(ans2,table_LoRaWAN_ANSWERS[10]);
    // create "invalid_param" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);
    
    if (status == 1)
    {
        _adr = true;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        _adr = false;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 3)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function is used to read the duty cycle prescaler from module
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getDutyCyclePrescaler()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get dcycle" command
    sprintf(_command,table_LoRaWAN_COMMANDS[31]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _dCyclePS = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*! 
 * @brief   This function pauses MAC functionality so the module is able
 *          to use radio functions.
 * 
 * @remarks This function must be called before using radio RX and TX
 *  
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t macPause()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac pause" command
    sprintf(_command,table_LoRaWAN_COMMANDS[9]);
    // create "4294967245" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[14]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}

/*! 
 * @brief   This function resumes MAC functionality
 *  
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t macResume()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac resume" command
    sprintf(_command,table_LoRaWAN_COMMANDS[10]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the frequency on the given channel ID
 * 
 * @param   uint32_t freq: frequency to be set [863250000..869750000]
 *                                             [433250000..434550000]
 *          uint8_t channel: channel to be set [3..15]
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer  
 *      @arg    '7' if input parameter error
 *  @arg    '8' if module does not support function
 */
uint8_t setChannelFreq(uint8_t channel, uint32_t freq)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    //check module (this function is only available for RN2483)
    if (_version == RN2903_MODULE)
    {
        return LORAWAN_VERSION_ERROR;
    }

    // check channel
    if (channel > 15 || channel <3) return LORAWAN_INPUT_ERROR;

    // check frequency settings
    if (freq < 433250000) return LORAWAN_INPUT_ERROR;
    if ((freq > 434550000)&&(freq < 863250000)) return LORAWAN_INPUT_ERROR;
    if (freq > 869750000) return LORAWAN_INPUT_ERROR;

    // clear buffers
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac set ch freq" command
    sprintf(_command,table_LoRaWAN_COMMANDS[20],channel,freq);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _freq[channel] = freq;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the operating frequency on the given channel
 *  
 * @param   uint8_t channel
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' if unrecognized module
 */
uint8_t getChannelFreq(uint8_t channel)
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));

    switch (_version)
    {
        case RN2483_MODULE:
                if (channel > 15)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        case RN2903_MODULE:
                if (channel > 71)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac get ch freq" command
    sprintf(_command,table_LoRaWAN_COMMANDS[35],channel);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        _freq[channel] = parseValue(10);
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the duty cycle on the given channel ID
 * 
 * @param   uint16_t dcycle: frequency to be set [0..65535]
 *          uint8_t channel: channel to be set [0..15]
 * 
 * @remarks The "dcycle" value that needs to be configured can be obtained 
 *          from the actual duty cycle X (in percentage) using the following formula:
 *          dcycle = (100/X) – 1
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *      @arg    '7' if input channel parameter error
 *  @arg    '8' module does not support function
 */
uint8_t setChannelDutyCycle(uint8_t channel, uint16_t dcycle)
{
    uint8_t status;
    float dutycycle;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    //check module (this function is only available for RN2483)
    if (_version == RN2903_MODULE)
    {
        return LORAWAN_VERSION_ERROR;
    }

    // check channel
    if (channel > 15) return LORAWAN_INPUT_ERROR;

    // create "mac set ch dcycle" command
    sprintf(_command,table_LoRaWAN_COMMANDS[21],channel,dcycle);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2(_command,ans1,ans2);
    
    if (status == 1)
    {
        _dCycle[channel] = dcycle;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function gets the operating duty channel on the given channel
 *  
 * @param   uint8_t channel
 * 
 * @remarks The "dcycle" value obtained from the module helps us to calculate
 *          duty cycle percentage "X" using the following formula:
 *          dcycle = (100/X) – 1
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '8' module does not support function
 */
uint8_t getChannelDutyCycle(uint8_t channel)
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));

    //check module (this function is only available for RN2483)
    if (_version == RN2903_MODULE)
    {
        return LORAWAN_VERSION_ERROR;
    }

    // check channel
    if (channel > 15) return LORAWAN_INPUT_ERROR;

    // create "mac get ch dcycle" command
    sprintf(_command,table_LoRaWAN_COMMANDS[36],channel);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        _dCycle[channel] = parseValue(10);
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function sets the data rate range on the given channel ID
 *
 * @param   uint8_t minDR: datarate to be set
 *          uint8_t maxDR: datarate to be set
 *          uint8_t channel: channel to be set
 * @remarks
 *          For RN2483:
 *                      minDR [0..5]
 *                      maxDR [0..5]
 *                      channel [0..15]
 *          For RN2903:
 *                      minDR [0..3]
 *                      maxDR [0..3]
 *                      channel [0..63]
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t setChannelDRRange(uint8_t channel, uint8_t minDR, uint8_t maxDR)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    switch (_version)
    {
        case RN2483_MODULE:
                if ((channel > 15) || (minDR > 5) || (maxDR > 5))
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        case RN2903_MODULE:
                if ((channel > 63) || (minDR > 3) || (maxDR > 3))
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac set ch drrange" command
    sprintf(_command,table_LoRaWAN_COMMANDS[22],channel,minDR,maxDR);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _drrMin[channel] = minDR;
        _drrMax[channel] = maxDR;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function gets the data rate range on the given channel
 *  
 * @param   uint8_t channel
 *      For RN2483: channel [0..15]
 *      For RN2903: channel [0..71]
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *      @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t getChannelDRRange(uint8_t channel)
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    switch (_version)
    {
        case RN2483_MODULE:
                if (channel > 15)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        case RN2903_MODULE:
                if (channel > 71)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac get ch drrange" command
    sprintf(_command,table_LoRaWAN_COMMANDS[37],channel);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        char * pch; 
        pch = strtok((char*) _buffer," \r\n");
        if (pch != NULL)
        {
            _drrMin[channel] = strtoul(pch,NULL, 10);
            pch = strtok(NULL," \r\n");
            if (pch != NULL)
            {
                _drrMax[channel] = strtoul(pch,NULL, 10);
                pch = strtok(pch," \r\n");
                return LORAWAN_ANSWER_OK;
            }
            return LORAWAN_ANSWER_ERROR;
        }       
        return LORAWAN_ANSWER_ERROR;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the status on the given channel ID
 * 
 * @param   char* state: state "on"/"off"
 *          uint8_t channel: channel to be set [0..15]
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t setChannelStatus(uint8_t channel, char* state)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // check state
    if ((strcmp(state, "on")) && (strcmp(state, "off"))) return LORAWAN_INPUT_ERROR;

    switch (_version)
    {
        case RN2483_MODULE:
                if (channel > 15)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        case RN2903_MODULE:
                if (channel > 71)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "mac set ch state" command
    sprintf(_command,table_LoRaWAN_COMMANDS[23],channel,state);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _status[channel] = state;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the status of the given channel
 *  
 * @param   uint8_t channel
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t getChannelStatus(uint8_t channel)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    switch (_version)
    {
        case RN2483_MODULE:
                if (channel > 15)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        case RN2903_MODULE:
                if (channel > 71)
                {
                    return LORAWAN_INPUT_ERROR;
                }
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac get ch status" command
    sprintf(_command,table_LoRaWAN_COMMANDS[38],channel);
    // create "on" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[9]);
    // create "off" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[10]);
    // create "invalid_param" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);
    
    if (status == 1)
    {
        _status[channel] = true;
        return LORAWAN_ANSWER_OK;
    }
    if (status == 2)
    {
        _status[channel] = false;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 3)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function is used to configure number of retransmisions
 *              for an uplink confirmed packet
 * 
 * @param   uint8_t retries: number of retries [0..255]
 *  
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t setRetries(uint8_t retries)
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "mac set retx" command
    sprintf(_command,table_LoRaWAN_COMMANDS[69],retries);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,100);
    
    if (status == 1)
    {
        _retries = retries;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*! 
 * @brief   This function is used to read the power index from module
 * 
 * @return      
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getRetries()
{
    uint8_t status; 
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get retx" command
    sprintf(_command,table_LoRaWAN_COMMANDS[70]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);
    
    if (status == 1)
    {
        _retries = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}






/*! 
 * @brief   This function gets current band of operation
 * 
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '8' module does not support function
 */
uint8_t getBand()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));

    //check module (this function is only available for RN2483)
    if (_version == RN2903_MODULE)
    {
        return LORAWAN_VERSION_ERROR;
    }

    // create "mac get band" command
    sprintf(_command,table_LoRaWAN_COMMANDS[28]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        memset(_band, 0x00, sizeof(_band));
        strncpy(_band,(char*)_buffer, sizeof(_band));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function gets the demodulation margin from the module
 *  
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getMargin()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get mrgn" command
    sprintf(_command,table_LoRaWAN_COMMANDS[32]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        uint8_t value = parseValue(10);
        _margin = value;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}

/*!
 * @brief   This function gets the number of gateways that successfully 
 *          received the last Linck Check Request from the module
 *  
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getGatewayNumber()
{
    uint8_t status;
    char ans1[50];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "mac get gwnb" command
    sprintf(_command,table_LoRaWAN_COMMANDS[33]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,300);
    
    if (status == 1)
    {
        _gwNumber = parseValue(10);
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the value of the uplink frame counter that will
 *          be used for the next uplink transmission.
 *
 * @param   uint8_t counter:
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t setUpCounter(uint32_t counter)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "mac set upctr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[72],counter);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);

    if (status == 1)
    {
        _upCounter = counter;
        saveConfig();
        return LORAWAN_ANSWER_OK;
    }
    else if(status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function is used to get the value of the uplink frame counter
 *          that will be used for the next uplink transmission.
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t getUpCounter()
{
    uint8_t status;
    char ans1[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));

    // create "mac get upctr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[73]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);

    if (status == 1)
    {
        _upCounter = parseValue(10);
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the value of the downlink frame counter that will
 *          be used for the next downlink transmission.
 *
 * @param   uint8_t counter:
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t setDownCounter(uint32_t counter)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "mac set dnctr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[74],counter);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);

    if (status == 1)
    {
        _downCounter = counter;
        saveConfig();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}

/*!
 * @brief   This function sets the time interval for the link check
 *          process to be triggered periodically
 *
 * @param   uint8_t counter:
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t setLinkCheck(uint16_t time)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "mac set dnctr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[77],time);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);

    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function is used to get the value of the downlink frame counter
 *          that will be used for the next downlink transmission.
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t getDownCounter()
{
    uint8_t status;
    char ans1[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));

    // create "mac get dnctr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[75]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);

    if (status == 1)
    {
        _downCounter = parseValue(10);
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Radio P2P functions
////////////////////////////////////////////////////////////////////////////////


/*!
 * @brief   Send a packet via radio
 * 
 * @param   char* message: char array that will be send
 * 
 * @remarks data is a sequence of digit representing the value of byte stream
 *          expressed in hexadecimal value (i.e. radio tx 12A435 – the payload 
 *          is composed by the following byte stream: 0x12, 0xA4, 0x35 – 6 digit
 *          converted in 3 bytes). The maximum length of frame is 510 digit (255 Bytes)
 *          for LoRa modulation and 128 digit (64 bytes) FSK modulation
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if there was an error
 *  @arg    '2' if no answer
 *      @arg    '7' if input parameter error
 */
uint8_t sendRadio(char * message)
{   
    char ans1[15];
    char ans2[15];
    uint8_t status;
    
    // clear buffers
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // check if payload is a hexadecimal string
    for (uint8_t i=0;i<strlen(message);i++)
    {
        if (((message[i] < 0x30) || (message[i] > 0x39)) && ((message[i] < 0x41) || (message[i] > 0x46)))
        {
            return LORAWAN_INPUT_ERROR;
        }
    }

    sprintf(_command,table_LoRaWAN_COMMANDS[40], message);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,600);

    if (status == 1)
    {
        memset(ans1,0x00,sizeof(ans1));
        memset(ans2,0x00,sizeof(ans2));
        
        // create "radio_tx_ok" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[4]);
        // create "radio_err" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[12]);
        
        //wait for response
        status = waitFor2_t(ans1,ans2,3000);
        
        if (status == 1)
        {
            return LORAWAN_ANSWER_OK;
        }
        else if (status == 2)
        {
            return LORAWAN_ANSWER_ERROR;
        }
        else
        {
            return LORAWAN_NO_ANSWER;
        }
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   Receive a packet via radio
 * 
 * @param   uint32_t timeout: time to wait for data
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if there was an error
 *  @arg    '2' it no answer
 *
 */
uint8_t receiveRadio(uint32_t timeout)
{
    uint8_t error;  
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    char ans3[15];
        
    //set watch dog radio to timeout
    error = setRadioWDT(timeout);
    if (error == 0)
    {
        //start radio receiving
        memset(_command,0x00,sizeof(_command));
        memset(ans1,0x00,sizeof(ans1));
        memset(ans2,0x00,sizeof(ans2));
        
        // create "radio rx" command
        sprintf(_command,table_LoRaWAN_COMMANDS[39]);
        // create "ok" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
        // create "invalid_param" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
        
        //send command and wait for ans
        status = sendCommand2_t(_command,ans1,ans2,100);
        
        if (status == 1)
        {
            memset(ans1,0x00,sizeof(ans1));
            memset(ans2,0x00,sizeof(ans2));
            
            // create "radio_rx  " answer
            sprintf(ans1,table_LoRaWAN_ANSWERS[7]);
            // create "\r\n" answer
            sprintf(ans2,table_LoRaWAN_ANSWERS[13]);
            // create "radio_err" answer
            sprintf(ans3,table_LoRaWAN_ANSWERS[12]);
            
            //wait for response
            status = waitFor2_t(ans1,ans3,timeout);

            if (status == 1)
            {
                //wait for response
                status = waitFor1_t(ans2,1000);
                
                _buffer[_length-1] = 0x00;
                _buffer[_length-2] = 0x00;
                _length -= 2;
                
                return LORAWAN_ANSWER_OK;
            }               
            return LORAWAN_NO_ANSWER;
        }
        else if (status == 2)
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    return LORAWAN_NO_ANSWER;
}
 
 
/*!
 * @brief   This function is used to radiate continuous wave without any modulation
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *      @arg    '2' it no answer
 */
uint8_t test_ON()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    sprintf(_command,table_LoRaWAN_COMMANDS[41]);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2(_command,ans1,ans2);
    
    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   Stops the continuous wave mode, this function works with module 
 *          identical to "reset()"
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *      @arg    '2' it no answer
 *
 */
uint8_t test_OFF()
{
    uint8_t status; 
    char ans1[15];
    char ans2[15];
    char ans3[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));

    // create "radio cw off" command
    sprintf(_command,table_LoRaWAN_COMMANDS[42]);
    // create "RN2483" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[5]);
    // create "RN2903" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[20]);
    // create "invalid_param" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand3(_command,ans1,ans2,ans3);

    if (status == 1)
    {
        _version = RN2483_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        _version = RN2903_MODULE;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 3)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function is used to read the SNR for the last receive packet
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t getRadioSNR()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get snr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[65]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);

    if (status == 1)
    {
        _radioSNR = parseValue(10);
        if (_radioSNR > 31)
        {
            _radioSNR = 64 - _radioSNR;
        }
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * 
 * @brief   This function sets spreading factor for radio mode
 *  
 * @param   char* sprfact:  spreading factor to be set [SF7..SF12]
 * 
 *  
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 */
uint8_t setRadioSF(char* sprfact)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "radio set sf" command
    sprintf(_command,table_LoRaWAN_COMMANDS[46],sprfact);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function gets the operating radio spreading factor from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioSF()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get sf" command
    sprintf(_command,table_LoRaWAN_COMMANDS[59]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        memset(_radioSF,0x00,sizeof(_radioSF));
        strncpy(_radioSF,(char*)_buffer,sizeof(_radioSF));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * 
 * @brief   This function sets tx power for radio mode
 *  
 * @param   uint8_t pwr: power to be set [-3..15]
 *  
 * @return       
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer 
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t setRadioPower(int8_t pwr)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    // create "radio set pwr" command
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    switch (_version)
    {
        case RN2483_MODULE:
                if ((pwr < -3) || (pwr > 15)) return LORAWAN_INPUT_ERROR;
                break;

        case RN2903_MODULE:
                if ((pwr < 2) || (pwr > 20)) return LORAWAN_INPUT_ERROR;
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    sprintf(_command,table_LoRaWAN_COMMANDS[45],pwr);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _radioPower = pwr;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function gets the operating radio power from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioPower()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get pwr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[58]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioPower = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the operating mode for transceiver use
 *  
 * @param   char* mode: "lora"/"fsk"
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioMode(char* mode)
{
    uint8_t status;
    char ans1[50];
    char ans2[50];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "radio set mod" command
    sprintf(_command,table_LoRaWAN_COMMANDS[43],mode);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,300);
    
    if (status == 1)
    {
        memset(_radioMode, 0x00, sizeof(_radioMode));
        strncpy(_radioMode,mode,sizeof(_radioMode));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the operating radio mode from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioMode()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get mod" command
    sprintf(_command,table_LoRaWAN_COMMANDS[56]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        memset(_radioMode,0x00,sizeof(_radioMode));
        strncpy(_radioMode, (char*)_buffer, sizeof(_radioMode));
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the operating frequency for transceiver use
 *  
 * @param   uint32_t freq: operating frequency [863250000..869750000]
 *              [433250000..434550000]
 *      uint32_t freq: operating frequency [923550000..927250000] (RN2903)
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t setRadioFreq(uint32_t freq)
{
    uint8_t status;
    char ans1[50];
    char ans2[50];

    switch (_version)
    {
        case RN2483_MODULE:
                if (freq < 433250000) return LORAWAN_INPUT_ERROR;
                if ((freq > 434550000)&&(freq < 863250000)) return LORAWAN_INPUT_ERROR;
                if (freq > 869750000) return LORAWAN_INPUT_ERROR;
                break;

        case RN2903_MODULE:
                if ((freq < 902250000)||(freq > 927750000)) return LORAWAN_INPUT_ERROR;
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "radio set freq" command
    sprintf(_command,table_LoRaWAN_COMMANDS[44],freq);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);

    if (status == 1)
    {
        _radioFreq = freq;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function gets the operating radio frequency from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioFreq()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get freq" command
    sprintf(_command,table_LoRaWAN_COMMANDS[57]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioFreq = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the signal bandwidth for receiver use
 *  
 * @param   float bandwidth: [250,125,62.5,31.3,15.6,7.8,3.9,200,100,50,25,
 *                            12.5,6.3,3.1,166.7,83.3,41.7,20.8,10.4,5.2,2.6]
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioReceivingBW(float bandwidth)
{
    uint8_t status;
    char bandw[6];
    char ans1[15];
    char ans2[15];
    char integer[4];
    char decimal[4];

    memset(_command,0x00,sizeof(_command));
    memset(bandw,0x00,sizeof(bandw));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(integer,0x00,sizeof(integer));
    memset(decimal,0x00,sizeof(decimal));

    //dtostrf (bandwidth,NULL,1,bandw);
    sprintf(bandw,"%f",bandwidth);
    
    char* pch = strtok(bandw,".\r\n");
    snprintf(integer, sizeof(integer), "%s",pch);
    pch = strtok(NULL,"\r\n");
    snprintf(decimal, sizeof(decimal), "%s",pch);
    
    if (decimal == "0")
    {
        // create "radio set rxbw" command
        sprintf(_command,table_LoRaWAN_COMMANDS[47],integer);
        // create "ok" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
        // create "invalid_param" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
        
        //send command and wait for ans
        status = sendCommand2_t(_command,ans1,ans2,500);
    }
    else
    {   
        // create "radio set rxbw" command
        sprintf(_command,table_LoRaWAN_COMMANDS[48],integer,decimal);
        // create "ok" answer
        sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
        // create "invalid_param" answer
        sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
        
        //send command and wait for ans
        status = sendCommand2_t(_command,ans1,ans2,500);
    }
        
    if (status == 1)
    {
        _radioRxBW = bandwidth;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the operating receiving bandwidth from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioReceivingBW()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get rxbw" command
    sprintf(_command,table_LoRaWAN_COMMANDS[60]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioRxBW = parseFloatValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the FSK bit rate value for transceiver use
 *  
 * @param   uint16_t bitrate: [0..65535]
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 */
uint8_t setRadioBitRateFSK(uint32_t bitrate)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    //check bit rate
    if ((bitrate > 300000)&&(bitrate < 1)) return LORAWAN_INPUT_ERROR;

    // create "radio set bitrate" command
    sprintf(_command,table_LoRaWAN_COMMANDS[49],bitrate);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _radioBitRate = bitrate;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function gets the bit rate for FSK modulation from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioBitRateFSK()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get bitrate" command
    sprintf(_command,table_LoRaWAN_COMMANDS[61]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioBitRate = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the frequency deviation for transceiver use
 *  
 * @param   uint16_t freqdeviation: [0..65535]
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 */
uint8_t setRadioFreqDeviation(uint32_t freqdeviation)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    //check freqdeviation
    if (freqdeviation > 200000) return LORAWAN_INPUT_ERROR;

    // create "radio set fdev" command
    sprintf(_command,table_LoRaWAN_COMMANDS[50],freqdeviation);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _radioFreqDev = freqdeviation;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}

/*!
 * @brief   This function gets the frequency deviation from the module
 *  
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioFreqDeviation()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get fdev" command
    sprintf(_command,table_LoRaWAN_COMMANDS[66]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioFreqDev = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the CRC header state for transceiver use
 *  
 * @param   uint16_t state: "on"/"off"
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioCRC(char* state)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "radio set crc" command
    sprintf(_command,table_LoRaWAN_COMMANDS[52],state);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        if (state == ans1) _crcStatus = true;
        else _crcStatus = false;
        
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2) 
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}

/*!
 * @brief   This function gets the CRC header state from module
 *  
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioCRC()
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    char ans3[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    memset(ans3,0x00,sizeof(ans3));
    
    // create "radio get crc" command
    sprintf(_command,table_LoRaWAN_COMMANDS[66]);
    // create "on" anwser
    sprintf(ans1,table_LoRaWAN_ANSWERS[9]);
    // create "off" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[10]);
    // create "invalid_param" answer
    sprintf(ans3,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand3_t(_command,ans1,ans2,ans3,500);
    
    if (status == 1)
    {
        _crcStatus = true;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        _crcStatus = false;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 3)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the preamble length for transceiver use
 *  
 * @param   uint16_t length: preamble length [0..65535]
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioPreamble(uint16_t length)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "radio set prlen" command
    sprintf(_command,table_LoRaWAN_COMMANDS[51],length);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _preambleLength = length;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the preamble length from module
 *  
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioPreamble()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get prlen" command
    sprintf(_command,table_LoRaWAN_COMMANDS[67]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _preambleLength = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function sets the coding rate for transceiver use
 *  
 * @param   char* codingrate: "4/5","4/6","4/7","4/8"
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioCR(char* codingrate)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "radio set cr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[53],codingrate);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        memset(_radioCR, 0x00, sizeof(_radioCR));
        strncpy(_radioCR,codingrate,sizeof(_radioCR));
        return LORAWAN_ANSWER_OK;
    }
    else  if (status == 2)
    {
        return LORAWAN_NO_ANSWER;
    }
    else
    {
        return LORAWAN_ANSWER_ERROR;
    }
}



/*!
 * @brief   This function gets the operating coding rate from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioCR()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get cr" command
    sprintf(_command,table_LoRaWAN_COMMANDS[62]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,100);
    
    if (status == 1)
    {
        char* pch = strtok((char*)_buffer,"\r\n");
        if (pch != NULL)
        {
            memset(_radioCR, 0x00, sizeof(_radioCR));
            strncpy(_radioCR, pch, sizeof(_radioCR));
            return LORAWAN_ANSWER_OK;
        }
        else
        {
            return LORAWAN_ANSWER_ERROR;
        }
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the time for the radio watch dog timer
 *  
 * @param   uint32_t time: [0..4294967295] time in milliseconds
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t setRadioWDT(uint32_t time)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));
    
    // create "radio set wdt" command
    sprintf(_command,table_LoRaWAN_COMMANDS[54],time);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _radioWDT = time;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function gets the watch dog timer's time from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioWDT()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get wdt" command
    sprintf(_command,table_LoRaWAN_COMMANDS[63]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioWDT = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}



/*!
 * @brief   This function sets the bandwidth for transceiver use
 *  
 * @param   uint16_t bandwitdh: 125,250,500
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 */
uint8_t setRadioBW(uint16_t bandwidth)
{
    uint8_t status;
    char ans1[15];
    char ans2[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    //check bandwidth
    if ((bandwidth == 125) || (bandwidth == 250) || (bandwidth == 500)){}
    else return LORAWAN_INPUT_ERROR;

    // create "radio set bw" command
    sprintf(_command,table_LoRaWAN_COMMANDS[55],bandwidth);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2_t(_command,ans1,ans2,500);
    
    if (status == 1)
    {
        _radioBW = bandwidth;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}




/*!
 * @brief   This function gets the operating radio bandwidth from module
 * 
 * @return  
 *  @arg    '0' if OK
 *  @arg    '1' if error 
 *  @arg    '2' if no answer
 */
uint8_t getRadioBW()
{
    uint8_t status;
    char ans1[15];
    
    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    
    // create "radio get bw" command
    sprintf(_command,table_LoRaWAN_COMMANDS[64]);
    // create "invalid_param" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[1]);
    
    //send command and wait for ans
    status = sendCommand2_t(_command,"\r\n",ans1,500);
    
    if (status == 1)
    {
        _radioBW = parseIntValue();
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else 
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets data rate and frecuency used for the
 *          second receive window.
 *
 * @remarks The configuration of the receive window parameters should
 *          be in concordance with the server configuration
 *
 * @param   uint8_t datarate: datarate to be set [0..5]
 *          uint32_t frequency: frequency to be set [863000000..870000000]
 *                                                  [433050000..434790000]
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 *  @arg    '7' if input parameter error
 *  @arg    '8' unrecognized module
 */
uint8_t setRX2Parameters(uint8_t datarate, uint32_t frequency)
{
    uint8_t status;
    float dutycycle;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    switch (_version)
    {
        case RN2483_MODULE:
                if (datarate > 7) return LORAWAN_INPUT_ERROR;
                if (frequency < 433250000) return LORAWAN_INPUT_ERROR;
                if ((frequency > 434550000)&&(frequency < 863250000)) return LORAWAN_INPUT_ERROR;
                if (frequency > 869750000) return LORAWAN_INPUT_ERROR;
                break;

        case RN2903_MODULE:
                if ((datarate > 13) || (datarate < 8))
                {
                    return LORAWAN_INPUT_ERROR;
                }
                if ((frequency < 923550000) || (frequency > 927250000)) return LORAWAN_INPUT_ERROR;
                break;

        default:
                return LORAWAN_VERSION_ERROR;
    }

    // create "mac set rx2" command
    sprintf(_command,table_LoRaWAN_COMMANDS[78],datarate,frequency);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2(_command,ans1,ans2);

    if (status == 1)
    {
        _rx2DataRate = datarate;
        _rx2Frequency = frequency;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


/*!
 * @brief   This function sets the delay used for the first receive window
 *
 * @param   uint16_t delay: delay to be set [0..65535]
 *
 * @remarks The "dcycle" value that needs to be configured can be obtained
 *          from the actual duty cycle X (in percentage) using the following formula:
 *          dcycle = (100/X) – 1
 *
 * @return
 *  @arg    '0' if OK
 *  @arg    '1' if error
 *  @arg    '2' if no answer
 */
uint8_t setRX1Delay(uint16_t delay)
{
    uint8_t status;
    float dutycycle;
    char ans1[15];
    char ans2[15];

    memset(_command,0x00,sizeof(_command));
    memset(ans1,0x00,sizeof(ans1));
    memset(ans2,0x00,sizeof(ans2));

    // create "mac set rx1delay" command
    sprintf(_command,table_LoRaWAN_COMMANDS[79],delay);
    // create "ok" answer
    sprintf(ans1,table_LoRaWAN_ANSWERS[0]);
    // create "invalid_param" answer
    sprintf(ans2,table_LoRaWAN_ANSWERS[1]);

    //send command and wait for ans
    status = sendCommand2(_command,ans1,ans2);

    if (status == 1)
    {
        _rx1Delay = delay;
        return LORAWAN_ANSWER_OK;
    }
    else if (status == 2)
    {
        return LORAWAN_ANSWER_ERROR;
    }
    else
    {
        return LORAWAN_NO_ANSWER;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief   This function parses a value 
 * @return  parsed value. '0' if nothing to parse
 */
uint32_t parseValue(uint8_t base)
{
    char * pch;
    pch = strtok((char*) _buffer," \r\n");
    if (pch != NULL)
    {
        return strtoul(pch,NULL, base);
    }
    return 0;
}


/*! 
 * @brief   This function parses a int value 
 * @return  parsed value. '0' if nothing to parse
 */         
uint32_t parseIntValue()
{
    char * pch; 
    pch = strtok((char*) _buffer,"\r\n");
    if (pch != NULL)
    {
        return atol(pch);
    }
    return 0;
}


/*! 
 * @brief   This function parses a float value
 * @return  parsed value. '0' if nothing to parse
 */
float parseFloatValue()
{
    char * pch; 
    pch = strtok((char*) _buffer,"\r\n");
    if (pch != NULL)
    {
        return atof(pch);
    }
    return 0;
}




// Preinstantiate Objects /////////////////////////////////////////////////////
//arduinoLoRaWAN LoRaWAN = arduinoLoRaWAN();
