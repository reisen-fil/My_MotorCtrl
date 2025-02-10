#ifndef __MASTER_PC_H
#define __MASTER_PC_H

#include "stm32f1xx_hal.h"

extern uint8_t Rx_Byte;			/* 接收到字节的存放变量 */
extern uint8_t Data_BitNum;		/* 接收到的实际数据位数 */

extern uint8_t id_Flag_1;		/* 参数数据包确定传参对象的标志1 */
extern uint8_t id_Flag_2;		/* 参数数据包确定传参对象的标志2 */
extern uint8_t PID_RxFlag;		/* PID参数获取标志位 */
extern uint8_t On_or_off_RxFlag;				/* 启动停止位获取标志位 */
extern uint8_t PID_index_1,PID_index_2;

extern uint8_t Get_Vofa_RxData[100];			/* 接收PID参数的数据包 */
extern uint8_t Get_On_or_off[100];						/* 获取启动位/停止位 */

extern int Car_on_off;					/* 小车启动停止位 */

typedef union Vofa_Receive
{
		float PID[3];
		uint8_t Car_Enable;
    
}VofaReceive_PID;

extern VofaReceive_PID PID_Mode[2];			/* 速度环/位置环 */

extern UART_HandleTypeDef huart1;

uint8_t Get_vofa_PID_Rxflag(void);  /* 获取数据接收成功标志位 */
uint8_t Get_On_or_off_Rxflag(void);	/* 获取启动停止位成功标志位 */
uint8_t Get_id_Flag_1(void);		/* 获取接收标志1 */
uint8_t Get_id_Flag_2(void);		/* 获取接收标志2 */

float RxPacket_Data_Handle(void);		/* 数据包换算处理 */

#endif

