#include "test.h"

/* 蓝牙调试部分 */

/*vscode使用printf重定向*/
int _write(int fd, char *pBuffer, int size)
{
    // for (int i = 0; i < size; i++)
    // {
    //     while ((USART1->SR & 0X40) == 0)
    //         ;                     //等待上一次串口数据发送完成
    //     USART1->DR = (u8)pBuffer; //写DR,串口1将发送数据
    // }
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == 0);
    HAL_UART_Transmit(&huart1, (uint8_t *)pBuffer, size, 0xff);
    return size;
}


