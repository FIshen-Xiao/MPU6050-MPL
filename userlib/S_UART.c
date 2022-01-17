#include "S_UART.h"

char Rx1Buffer[RXBUFFERSIZE]; //接收数据
char Rx2Buffer[RXBUFFERSIZE]; //接收数据
uint8_t aRx1Buffer;			  //接收中断缓冲
uint8_t aRx2Buffer;			  //接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;	  //串口1接收缓冲计数
uint8_t Uart2_Rx_Cnt = 0;	  //串口2接收缓冲计数
uint8_t Uart1RXFlag = 0;	  //串口1接收完成标志

void UART_printf(UART_HandleTypeDef *uartbase, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int length;
	char buffer[128];
	length = vsnprintf(buffer, 128, fmt, ap);
	HAL_UART_Transmit(uartbase, (uint8_t *)buffer, length, 0xffff);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	// UNUSED(huart);
	/* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if (Uart1_Rx_Cnt >= 255) //溢出判断
	{
		Uart1_Rx_Cnt = 0;
		memset(Rx1Buffer, 0x00, sizeof(Rx1Buffer)); //清空数组
		Uart1RXFlag = 0;
	}
	else if (Uart1RXFlag == 0)
	{
		Rx1Buffer[Uart1_Rx_Cnt++] = aRx1Buffer;												//接收数据转存
		if ((Rx1Buffer[Uart1_Rx_Cnt - 1] == 0x0A) && (Rx1Buffer[Uart1_Rx_Cnt - 2] == 0x0D)) //判断结束位
		{
			Uart1RXFlag = 1;
			Uart1_Rx_Cnt = 0;
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRx1Buffer, 1); //再开启接收中断
}