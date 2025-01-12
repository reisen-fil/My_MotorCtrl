#ifndef __TEST_H
#define __TEST_H

#include "stm32f1xx_hal.h"
#include <stdio.h> 
#include "usart.h"
#include "math.h"

// int fputc(int ch, FILE *f);     /* 重定向打印函数 */
int _write(int fd, char *pBuffer, int size);

#endif

