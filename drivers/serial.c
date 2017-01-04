/* Scheduler includes. */
#include "FreeRTOS.h"

/* Library includes. */
#include "stm32f10x.h"

#include "serial.h"
/*-----------------------------------------------------------*/

static USART_TypeDef* get_device(SerialUsart_t usart)
{
	switch (usart)
	{
	case SERIAL_USART_1: return USART1;
	case SERIAL_USART_2: return USART2;
	case SERIAL_USART_3: return USART3;
	}
	return 0;
}

int comm_test(SerialUsart_t usart)
{
        return ( USART_GetFlagStatus(get_device(usart), USART_FLAG_RXNE) == RESET ) ? 0 : 1;
}

char comm_get(SerialUsart_t usart)
{
        while(USART_GetFlagStatus(get_device(usart), USART_FLAG_RXNE) == RESET) { ; }
        return (char)USART_ReceiveData(get_device(usart));
}

void comm_put(SerialUsart_t usart, char d)
{
        while(USART_GetFlagStatus(get_device(usart), USART_FLAG_TXE) == RESET) { ; }
        USART_SendData(get_device(usart), (uint16_t)d);
}

void comm_puts(SerialUsart_t usart, const char* s)
{
        char c;
        while ( ( c = *s++) != '\0' ) {
                comm_put(usart, c);
        }
}

int comm_get_line(SerialUsart_t usart, char *out, int out_size)
{
	int ii = 0;
	for (ii = 0; ii < out_size - 1; ii++)
	{
		out[ii] = comm_get(usart);
		printf("%d char %d %c\r\n", ii, out[ii], out[ii]);
		if (out[ii] == '\r' || out[ii] == '\n')
			break;
	}
	out[ii] = 0;
	return ii;
}


void serial_init(SerialUsart_t usart, uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t tx_pin;
	uint16_t rx_pin;
	GPIO_TypeDef *port = GPIOA;

	USART_InitTypeDef USART_InitStructure;
	USART_TypeDef* usart_device;

	//enable bus clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	if (usart == SERIAL_USART_1)
	{
		usart_device = USART1;
		tx_pin = GPIO_Pin_9;
		rx_pin = GPIO_Pin_10;
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
	}
	else if (usart == SERIAL_USART_2)
	{
		usart_device = USART2;
		tx_pin = GPIO_Pin_2;
		rx_pin = GPIO_Pin_3;
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE);
	}
	else if (usart == SERIAL_USART_3)
	{
		usart_device = USART3;
		tx_pin = GPIO_Pin_10;
		rx_pin = GPIO_Pin_11;
		port = GPIOB;
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE);
	}

	//Configure USART1 Tx (PA.02) as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin = tx_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
	//Configure USART1 Rx (PA.03) as input floating
	GPIO_InitStructure.GPIO_Pin = rx_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(port, &GPIO_InitStructure);

	/* USART1 and USART2 configuration ------------------------------------------------------*/
	/* USART and USART2 configured as follow:
         - BaudRate = 115200 baud
         - Word Length = 8 Bits
         - One Stop Bit
         - No parity
         - Hardware flow control disabled (RTS and CTS signals)
         - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USART2 */
	USART_Init(usart_device, &USART_InitStructure);

	/* Enable the USART1 */
	USART_Cmd(usart_device, ENABLE);
}




