
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCOM_H__
#define __VCOM_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include <usart.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/** 
* @brief  Init the VCOM.
* @param  uartHandle
* @return None
*/
 void vcom_Init(UART_HandleTypeDef uartHandle);

   /** 
* @brief  DeInit the VCOM.
* @param  None
* @return None
*/
void vcom_DeInit(void);

   /** 
* @brief  sends string on com port
* @param  string
* @return None
*/
void vcom_Send( char *format, ... );

/* Exported macros -----------------------------------------------------------*/
#if 1
#define PRINTF(...)     vcom_Send(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#ifdef __cplusplus
}
#endif

#endif /* __VCOM_H__*/
