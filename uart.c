#include "avr/io.h"
#include "avr/interrupt.h"
#include "uart.h"

/* Global variables to hold the address of the call back function in the application */
static volatile void (*int_callBackPtr)(void) = NULL_PTR;

ISR(USART0_RX_vect){
	if(int_callBackPtr != NULL_PTR)
	{
		/* Call the Call Back function in the application after the edge is detected */
		(*int_callBackPtr)();
	}
}

void USART_INT_EN(){
	UCSR0B |= (1 << RXCIE0); //recieve data interrupt, makes sure we don't loose data
}
void USART_INT_DIS(){
	UCSR0B &= ~(1 << RXCIE0); //recieve data interrupt, makes sure we don't loose data
}
void uart_start(void) {
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //transmit side of hardware
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); //receive side of hardware

  UBRR0L = BAUD_PRESCALE; //set the baud to 9600, have to split it into the two registers
  UBRR0H = (BAUD_PRESCALE >> 8); //high end of baud register

  USART_INT_DIS();

  #if DEBUG
    uart_sendstr("0x04 - UART is up...");
  #endif
}

void uart_sendint(uint8_t data) {
    /*
    Use this to send a 8bit long piece of data
    */
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = data; //send the data
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = '\r';//send a new line just to be sure
}

void uart_sendint16(uint16_t data) {
    /*
    Use this to send a 16bit long piece of data
    */
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = data;//send the lower bits
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = (data >> 8); //send the higher bits
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = '\n';//send a new line just to be sure
}

void uart_sendstr(char *data) {
    /*
    Use this to send a string, it will split it up into individual parts
    send those parts, and then send the new line code
    */
    while (*data) {
        while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
        UDR0 = *data; //goes through and splits the string into individual bits, sends them
        data += 1;//go to new bit in string
    }
    while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
    UDR0 = '\r';//send a new line just to be sure
}


//
//ISR(SIG_USART_RECV) {//sets up the interrupt to recieve any data coming in
//	//	/* RXC flag is set when the UART receive data so wait until this
//	//	 * flag is set to one */
//	//	while(BIT_IS_CLEAR(UCSR1A,RXC)){}
//	//	/* Read the received data from the Rx buffer (UDR) and the RXC flag
//	//	   will be cleared after read this data */
//	//    return UDR1;
//}



//#include "uart_private.h"
//#include "uart.h"
//#include "../../common_macros.h"
//#include "avr/io.h"
// /*
//four uarts
//u0 ->
//u1 ->
//u2 ->
//u4 ->
//
//RX2 - PH0
//TX2 - PE1
//
//RX3 - PC9
//TX3 - PC10
//
//RX1 - PD2
//TX1 - PD3
//
//RX0 - PE0
//TX0 - PE1
//*/
//
//
//#define BAUD_PRESCALE	((F_CPU /( 16 *USART_BAUDRATE ))- 1)//(((F_CPU / (USART_BAUDRATE * 8UL))) - 1)
//
//// UCSR3A
//#define MPCM		0
//#define U2X			1
//#define UPE			2
//#define DOR			3
//#define FE			4
//#define UDRE		5
//#define TXC			6
//#define RXC			7
//
////UCSR3B
//#define	TXB8		0
//#define RXB8		1
//#define UCXZ2		2
//#define	TXEN		3
//#define	RXEN		4
//#define	UDRIE		5
//#define	TXCIE		6
//#define	RXCIE		7
//
////UCSR3C
//#define	UCPOL		0
//#define UCSZ0		1
//#define UCSZ1		2
//#define	USBS		3
//#define	UPM0		4
//#define	UPM1		5
//#define	UMSEL0		6
//#define	UMSEL1		7
//
//void USART_Init( ){
//	//unsigned int BAUD_PRESCALE = 52;
//	/************************** UCSRB Description **************************
//	 * RXCIE = 0 Disable USART RX Complete Interrupt Enable
//	 * TXCIE = 0 Disable USART TX Complete Interrupt Enable
//	 * UDRIE = 0 Disable USART Data Register Empty Interrupt Enable
//	 * RXEN  = 1 Receiver Enable
//	 * RXEN  = 1 Transmitter Enable
//	 * UCSZ2 = 0 For 8-bit data mode
//	 * RXB8 & TXB8 not used for 8-bit data mode
//	 ***********************************************************************/
//	UCSR1B = (1<<RXEN) | (1<<TXEN);
//
//	/************************** UCSRC Description **************************
//	 * URSEL1   = 0 FOR using UART
//	 * URSEL0	= 0 FOR Async UART
//	 * UPM1:0  = 00 Disable parity bit
//	 * USBS    = 0 One stop bit
//	 * UCSZ1:0 = 11 For 8-bit data mode
//	 * UCPOL   = 0 Used with the Synchronous operation only
//	 ***********************************************************************/
//	UCSR1C = (1<<UCSZ0) | (1<<UCSZ1);
//
//	/* First 8 bits from the BAUD_PRESCALE inside UBRRL and last 4 bits in UBRRH*/
//	UBRR1H = BAUD_PRESCALE>>8;
//	UBRR1L = BAUD_PRESCALE;
//} // USART_Init
//
//void USART_Transmit( unsigned char data )
//{
//	/* UDRE flag is set when the Tx buffer (UDR) is empty and ready for
//	 * transmitting a new byte so wait until this flag is set to one */
//	while ( BIT_IS_CLEAR(UCSR1A,UDRE));
//	/* Put the required data in the UDR register and it also clear the UDRE flag as
//	 * the UDR register is not empty now */
//	UDR1 = data;
//
//	/************************* Another Method *************************
//	UDR0 = data;
//	while(BIT_IS_CLEAR(UCSR0A,TXC)){} // Wait until the transimission is complete TXC = 1
//	SET_BIT(UCSR0A,TXC); // Clear the TXC flag
//	*******************************************************************/
//}
//
//
unsigned char USART_Receive( void )
{
	/* RXC flag is set when the UART receive data so wait until this
	 * flag is set to one */
	while(BIT_IS_CLEAR(UCSR0A,RXC0)){}
	/* Read the received data from the Rx buffer (UDR) and the RXC flag
	   will be cleared after read this data */
    return UDR0;
}

void int_setCallBack(void(*a_ptr)(void))
{
	/* Save the address of the Call back function in a global variable */
	int_callBackPtr = a_ptr;
}