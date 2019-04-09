#include "main.h"

void USART_INIT(uint16_t RX, uint16_t TX, GPIO_TypeDef * RX_GPIO, GPIO_TypeDef * TX_GPIO, int baud);

void TRANSMIT_STR(char c[]);

void TRANSMIT_CHAR(char c);

void USART1_Enable(void);

void SET_ALT_FUNC_USART1(void);
