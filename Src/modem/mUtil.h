/*! \file arduinoUART.h
    \brief Library for managing UART bus
    
    Copyright (C) 2015 Libelium Comunicaciones Distribuidas S.L.
    http://www.libelium.com
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 2.1 of the License, or
    (at your option) any later version.
   
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
  
    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
    Version:        1.5
    Design:         David Gascón
    Implementation: Yuri Carmona, Ruben Martin
*/


  
#ifndef mUtil_h
#define mUtil_h

/******************************************************************************
 * Includes
 ******************************************************************************/

#include <inttypes.h>
#include <stdbool.h>

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/
 
 
/*! \def DEF_COMMAND_TIMEOUT
    \brief default timeout for command operations
 */
#define DEF_COMMAND_TIMEOUT  5000

/*! \def DEF_COMMAND_DELAY
    \brief default time to wait after sending command
 */
#define DEF_COMMAND_DELAY    100
 
/*! \def DEF_BAUD_RATE
    \brief default baudrate
 */
#define DEF_BAUD_RATE    115200



/**
  * @brief  TIMER State structures definition
  */
typedef enum
{
  TIMER_STATE_STOP				 = 0x00,
  TIMER_STATE_RUN				 = 0x01,
  TIMER_STATE_TIMEOUT			 = 0x02,
  TIMER_STATE_ERROR				 = 0x03
}mUtil_Timer_State;


mUtil_Timer_State startTimer(uint32_t timeout);
mUtil_Timer_State stopTimer();
mUtil_Timer_State statusTimer();
void mUtil_Timer_Callback();
void mUtil_InitUart();
void mUtil_Uart_Callback();

//! buffer for rx data
uint8_t _buffer[256];

//! length of the contents in '_buffer'
uint16_t _length;

//! It sends a command through the selected uart expecting a specific answer
/*!
\param char* command : string to send to the module
\param char* ans1 : string expected to be answered by the module
\return '0' if timeout error,
		'1' if ans1
*/
uint8_t sendCommand1(char* command,
					char* ans1);

uint8_t sendCommand1_t(char* command,
					char* ans1,
					uint32_t timeout);

//! It sends a command through the selected uart expecting specific answers
/*!
\param char* command : string to send to the module
\param char* ans1 : string expected to be answered by the module
\param char* ans2 : string expected to be answered by the module
\param uint32_t timeout : time to wait for responses before exit with error
\return '0' if timeout error,
		'1' if ans1
		'2' if ans2
 */
uint8_t sendCommand2(char* command,
					char* ans1,
					char* ans2);

uint8_t sendComman2_t(char* command,
					char* ans1,
					char* ans2,
					uint32_t timeout);

//! It sends a command through the selected uart expecting specific answers
/*!
\param char* command : string to send to the module
\param char* ans1 : string expected to be answered by the module
\param char* ans2 : string expected to be answered by the module
\param char* ans3 : string expected to be answered by the module
\param uint32_t timeout : time to wait for responses before exit with error
\return '0' if timeout error,
		'1' if ans1
		'2' if ans2
		'3' if ans3
 */
uint8_t sendCommand3(char* command,
					char* ans1,
					char* ans2,
					char* ans3 );

uint8_t sendCommand3_t(char* command,
					char* ans1,
					char* ans2,
					char* ans3,
					uint32_t timeout);

//! It sends a command through the selected uart expecting specific answers
/*!
\param char* command : string to send to the module
\param char* ans1 : string expected to be answered by the module
\param char* ans2 : string expected to be answered by the module
\param char* ans3 : string expected to be answered by the module
\param char* ans4 : string expected to be answered by the module
\param uint32_t timeout : time to wait for responses before exit with error
\return '0' if timeout error,
		'1' if ans1
		'2' if ans2
		'3' if ans3
		'4' if ans4
 */
uint8_t sendCommand4(char* command,
					char* ans1,
					char* ans2,
					char* ans3,
					char* ans4);

uint8_t sendCommand4_t(char* command,
					char* ans1,
					char* ans2,
					char* ans3,
					char* ans4,
					uint32_t timeout);

//! It seeks 'pattern' inside the 'buffer' array
bool find(uint8_t* buffer, uint16_t length, char* pattern);

//! It sends a command without waiting answer (only send)
void sendCommand0(uint8_t* command, uint16_t length);

/*!
\brief  This function waits for one of the answers during a certain period
		of time. The result is stored in '_buffer'.
\param  char* command: command to be sent
\param  char* ans1: expected answer
\param  char* ans2: expected answer
\param  char* ans3: expected answer
\param  char* ans4: expected answer
\param  uint32_t timeout : time to wait for responses before exit with error
\return '0' if timeout error,
		'1' if ans1
		'2' if ans2
		'3' if ans3
		'4' if ans4
*/
uint8_t waitFor1( char* ans1);
uint8_t waitFor1_t( char* ans1, uint32_t timeout);
uint8_t waitFor2( char* ans1, char* ans2);
uint8_t waitFor2_t( char* ans1, char* ans2, uint32_t timeout);
uint8_t waitFor3( char* ans1, char* ans2, char* ans3 );
uint8_t waitFor3_t( char* ans1, char* ans2, char* ans3, uint32_t timeout);
uint8_t waitFor4( char* ans1, char* ans2, char* ans3, char* ans4);
uint8_t waitFor4_t( char* ans1, char* ans2, char* ans3, char* ans4, uint32_t timeout);

#endif
