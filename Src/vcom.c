  
#include "vcom.h"
#include <stdarg.h>
#include <stdint.h>
#include "usart.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 128

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t iw;
static char buff[BUFSIZE+16];
static UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void vcom_Init(UART_HandleTypeDef uartHandle)
{
	UartHandle = uartHandle;
}


void vcom_DeInit(void)
{

}

void vcom_Send( char *format, ... )
{
  va_list args;
  va_start(args, format);
  
  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);
  
  HAL_UART_Transmit(&UartHandle,(uint8_t *)&buff[0], iw, 300);
  
  va_end(args);
}
