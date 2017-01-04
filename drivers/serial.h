#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <stdint.h>

typedef enum 
{ 
	ser50,		
	ser75,		
	ser110,		
	ser134,		
	ser150,    
	ser200,
	ser300,		
	ser600,		
	ser1200,	
	ser1800,	
	ser2400,   
	ser4800,
	ser9600,		
	ser19200,	
	ser38400,	
	ser57600,	
	ser115200
} eBaud;

typedef enum
{
	SERIAL_USART_1, // connected to USB
	SERIAL_USART_2,
	SERIAL_USART_3
} SerialUsart_t;

void comm_put(SerialUsart_t usart, char d);
void comm_puts(SerialUsart_t usart, const char* s);
void serial_init(SerialUsart_t usart, uint32_t speed);
char comm_get(SerialUsart_t usart);
int  comm_get_line(SerialUsart_t usart, char *out, int out_size);

#endif

