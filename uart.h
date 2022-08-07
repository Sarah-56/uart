#ifndef MCAL_UART_UART_H_
#define MCAL_UART_UART_H_
#include "../../Utils/std_types.h"
#include "../../Utils/common_macros.h"

/*******************************************************************************
 *                      Preprocessor Macros                                    *
 *******************************************************************************/

//-------------------------------------------
//Macros (defines)
//-------------------------------------------
#define BAUD 9600
#define BUFF_LEN 700
#define BAUD_PRESCALE (((F_CPU / (BAUD * 16UL))) - 1)
#define DEBUG 0
//-------------------------------------------
//Prototypes
//-------------------------------------------
void SWUART_init(uint32_t baudrate);
void SWUART_send(uint8_t data);
void SWUART_recieve(uint8_t *data);

void USART_INT_EN();
void USART_INT_DIS();
void uart_start(void);
void uart_sendint(uint8_t data);
void uart_sendint16(uint16_t data);
void uart_sendstr(char *data);
uint8_t uart_get(void);

//-------------------------------------------
//Variables
//-------------------------------------------
char input_buffer[BUFF_LEN];

uint16_t read_spot;


/* UART Driver Baud Rate */
#define USART_BAUDRATE 9600UL

void USART_Init( );

void USART_Transmit( unsigned char data );

unsigned char USART_Receive(void);

void int_setCallBack(void(*a_ptr)(void));

#endif /* MCAL_UART_UART_H_ */