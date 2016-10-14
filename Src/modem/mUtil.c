/*
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
 *  Version:    1.5
 *  Design:   David Gascón
 *  Implementation: Yuri Carmona, Ruben Martin
 */

#include <string.h>
#include "stm32l4xx_hal.h"
#include "mUtil.h"
#include "../vcom.h"

char Rx_index, Rx_data, Rx_Buffer[256], Transfer_cplt;

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;


/*
 * 
 * name: sendCommand
 * @param char* command: command to be sent
 * @param char* ans1: expected answer
 * @param bool flush: flush needed before sending command (1: flush; 0: not)
 * @return  '0' if timeout error, 
 *      '1' if ans1
 * 
 */
uint8_t sendCommand1( char* command,
                char* ans1 )
{
  return sendCommand4_t(command, ans1, NULL, NULL, NULL, DEF_COMMAND_TIMEOUT );
}

uint8_t sendCommand1_t( char* command,
                char* ans1, 
                uint32_t timeout)
{
  return sendCommand4_t(command, ans1, NULL, NULL, NULL, timeout );
}


uint8_t sendCommand2( char* command,
                char* ans1, 
                char* ans2 )
{
  return sendCommand4_t(command, ans1, ans2, NULL, NULL, DEF_COMMAND_TIMEOUT );
}
    
    
uint8_t sendCommand2_t( char* command,
                char* ans1, 
                char* ans2, 
                uint32_t timeout) 
{
  return sendCommand4_t(command, ans1, ans2, NULL, NULL, timeout );
}       
  

uint8_t sendCommand3( char* command,
                char* ans1, 
                char* ans2, 
                char* ans3 )
{
  return sendCommand4_t(command, ans1, ans2, ans3, NULL, DEF_COMMAND_TIMEOUT );
}       
            

uint8_t sendCommand3_t( char* command,
                char* ans1, 
                char* ans2,
                char* ans3,  
                uint32_t timeout)
{
  return sendCommand4_t(command, ans1, ans2, ans3, NULL, timeout );
}       
      
  
uint8_t sendCommand4( char* command,
                char* ans1, 
                char* ans2, 
                char* ans3, 
                char* ans4)
{
  return sendCommand4_t(command, ans1, ans2, ans3, ans4, DEF_COMMAND_TIMEOUT );
}   


/*
 * 
 * name: sendCommand
 * @param char* command: command to be sent
 * @param char* ans1: expected answer
 * @param char* ans2: expected answer
 * @param char* ans3: expected answer
 * @param char* ans4: expected answer
 * @param uint32_t timeout: time to wait for response
 * @return  '0' if timeout error, 
 *      '1' if ans1
 *      '2' if ans2
 *      '3' if ans3
 *      '4' if ans4
 * 
 */
uint8_t  sendCommand4_t(  char* command,
                char* ans1, 
                char* ans2, 
                char* ans3, 
                char* ans4, 
                uint32_t timeout)
{
  // index counter
  uint16_t i = 0;
        
  /// 1. print command
//    printString( command, _uart );
	HAL_UART_Transmit(&huart3, (uint8_t*) command, strlen(command), 1000);

    HAL_Delay( DEF_COMMAND_DELAY );
  
  
  /// 2. read answer  
  // clear _buffer
  memset( _buffer, 0x00, sizeof(_buffer) );
  _length = 0;
  
  Transfer_cplt = 0;

  HAL_UART_Transmit(&huart1, (uint8_t*) command, strlen(command), 1000);
  startTimer(timeout);

  while( (Transfer_cplt == 0) && (statusTimer() == TIMER_STATE_RUN) );

  if(statusTimer() == TIMER_STATE_TIMEOUT)
	  return 0; // timeout
  else
	  stopTimer(); // stop timer

//  // check available data for 'timeout' milliseconds
//    while( (millis() - previous) < timeout )
//    {
//
//
//    if( serialAvailable(_uart) )
//    {
//      if ( i < (sizeof(_buffer)-1) )
//      {
//        _buffer[i++] = serialRead(_uart);
//        _length++;
//      }
//    }
      
    // Check 'ans1'
    if( find( _buffer, _length, ans1 ) == true )
    {   
      return 1;
    }
    
    // Check 'ans2'
    if( ans2 != NULL )
    {
      if( find( _buffer, _length, ans2 ) == true )
      { 
        return 2;
      }
    }
    
    // Check 'ans3'
    if( ans3 != NULL )
    {
      if( find( _buffer, _length, ans3 ) == true )
      { 
        return 3;
      }
    }
    
    // Check 'ans4'
    if( ans4 != NULL )
    {
      if( find( _buffer, _length, ans4 ) == true )
      { 
        return 4;
      }
    } 
    
    
  // timeout
  return 0; 
  
}




/*
 * 
 * name: find
 * @param uint8_t* buffer: pointer to buffer to be scanned
 * @param uint16_t length: actual length of buffer
 * @param char* pattern: pattern to find
 * @return  '0' not found, 
 *      '1' pattern found
 * 
 */
bool find( uint8_t* buffer, uint16_t length, char* pattern)
{
  int result;
  
  if( length >= strlen(pattern) )
  {   
    for(uint16_t i = 0; i <= length-strlen(pattern); i++)
    {
      result = memcmp( &buffer[i], pattern, strlen(pattern) );
      
      if( result == 0 )
      {
        return true;
      }
    }
  }
  
  return false;
}






/*
 * 
 * name: sendCommand
 * @param uint8_t* command: pointer to the buffer with the command to be sent
 * @param uint16_t length: length of the buffer
 * @return  void 
 */
void  sendCommand0( uint8_t* command, uint16_t length )
{   
	HAL_UART_Transmit(&huart3, (uint8_t*) command, strlen(command), 1000);

	HAL_Delay( DEF_COMMAND_DELAY );
}






/*
 * 
 * name: sendCommand
 * @param char* command: command to be sent
 * @param char* ans1: expected answer
 * @param bool flush: flush needed before sending command (1: flush; 0: not)
 * @return  '0' if timeout error, 
 *      '1' if ans1
 * 
 */
uint8_t waitFor1( char* ans1 )
{
  return waitFor4_t( ans1, NULL, NULL, NULL, DEF_COMMAND_TIMEOUT );
}

uint8_t waitFor1_t( char* ans1,
              uint32_t timeout)
{
  return waitFor4_t( ans1, NULL, NULL, NULL, timeout );
}


uint8_t waitFor2( char* ans1,
              char* ans2 )
{
  return waitFor4_t( ans1, ans2, NULL, NULL, DEF_COMMAND_TIMEOUT );
}
    
    
uint8_t waitFor2_t( char* ans1,
              char* ans2, 
              uint32_t timeout) 
{
  return waitFor4_t( ans1, ans2, NULL, NULL, timeout );
}       
  

uint8_t waitFor3( char* ans1,
              char* ans2, 
              char* ans3 )
{
  return waitFor4_t( ans1, ans2, ans3, NULL, DEF_COMMAND_TIMEOUT );
}       
            

uint8_t waitFor3_t( char* ans1,
              char* ans2,
              char* ans3,  
              uint32_t timeout)
{
  return waitFor4_t( ans1, ans2, ans3, NULL, timeout );
}       
      
  
uint8_t waitFor4( char* ans1,
              char* ans2, 
              char* ans3, 
              char* ans4)
{
  return waitFor4_t( ans1, ans2, ans3, ans4, DEF_COMMAND_TIMEOUT );
}   


/*
 * 
 * name: waitFor
 * @brief This function waits for one of the answers during a certain period 
 *      of time. The result is stored in '_buffer'.
 * @param char* command: command to be sent
 * @param char* ans1: expected answer
 * @param char* ans2: expected answer
 * @param char* ans3: expected answer
 * @param char* ans4: expected answer
 * @return  '0' if timeout error, 
 *      '1' if ans1
 *      '2' if ans2
 *      '3' if ans3
 *      '4' if ans4
 */
uint8_t  waitFor4_t( char* ans1,
              char* ans2, 
              char* ans3, 
              char* ans4, 
              uint32_t timeout )
{ 
  // index counter
  uint16_t i = 0;
  
  // clear _buffer
  memset( _buffer, 0x00, sizeof(_buffer) );
  _length = 0;
  
  Transfer_cplt = 0;

  startTimer(timeout);
  
  // check available data for 'timeout' milliseconds
    while( statusTimer() == TIMER_STATE_RUN )
    {
    	while( (Transfer_cplt == 0) && (statusTimer() == TIMER_STATE_RUN) );
    	Transfer_cplt = 0;
      
    // Check 'ans1'
    if( find( _buffer, _length, ans1 ) == true )
    {   
    	stopTimer(); // stop timer
      return 1;
    }
    
    // Check 'ans2'
    if( ans2 != NULL )
    {
      if( find( _buffer, _length, ans2 ) == true )
      { 
    	  stopTimer(); // stop timer
        return 2;
      }
    }
    
    // Check 'ans3'
    if( ans3 != NULL )
    {
      if( find( _buffer, _length, ans3 ) == true )
      { 
    	  stopTimer(); // stop timer
        return 3;
      }
    }
    
    // Check 'ans4'
    if( ans4 != NULL )
    {
      if( find( _buffer, _length, ans4 ) == true )
      { 
    	  stopTimer(); // stop timer
        return 4;
      }
    } 
    
  }

  	  stopTimer(); // stop timer
    
  // timeout
  return 0; 

}


/////////////////////////////////////////////////////////////////////////


mUtil_Timer_State timer_state = TIMER_STATE_STOP;

mUtil_Timer_State startTimer(uint32_t timeout)
{
	stopTimer();

	htim2.Init.Period = timeout - 1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
	  return TIMER_STATE_ERROR;
	}

	HAL_TIM_Base_Start_IT(&htim2);
	timer_state = TIMER_STATE_RUN;
	return timer_state;
}

mUtil_Timer_State stopTimer()
{
	if(timer_state == TIMER_STATE_RUN)
		 HAL_TIM_Base_Stop_IT(&htim2);
	timer_state = TIMER_STATE_STOP;
	return timer_state;
}

mUtil_Timer_State statusTimer()
{
	return timer_state;
}

inline void mUtil_Timer_Callback()
{
	stopTimer();
	timer_state = TIMER_STATE_TIMEOUT;
}

void mUtil_InitUart()
{
	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
}

inline void mUtil_Uart_Callback()
{
	Rx_Buffer[Rx_index++] = Rx_data;	//add data to Rx_Buffer

	if (Rx_data == 10)			//if received data = '\n'
	{
		Rx_Buffer[Rx_index] = '\0';
		Rx_index = 0;
		strcpy(_buffer, Rx_Buffer);
		_length = strlen(_buffer);
		Transfer_cplt = 1;			//transfer complete, data is ready to read

		PRINTF("%s", Rx_Buffer);
	}

	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);	//activate UART receive interrupt every time
}

