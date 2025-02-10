#include "Master_PC.h"
// #include "Variable.h"


uint8_t Rx_Byte;			/* 接收到字节的存放变量 */
uint8_t Data_BitNum;		/* 接收到的实际数据位数 */

uint8_t id_Flag_1;		/* 参数数据包确定传参对象的标志1 */
uint8_t id_Flag_2;		/* 参数数据包确定传参对象的标志2 */
uint8_t PID_RxFlag;		/* PID参数获取标志位 */
uint8_t On_or_off_RxFlag;				/* 启动停止位获取标志位 */
uint8_t PID_index_1,PID_index_2;

uint8_t Get_Vofa_RxData[100];			/* 接收PID参数的数据包 */
uint8_t Get_On_or_off[100];						/* 获取启动位/停止位 */

int Car_on_off;					/* 小车启动停止位 */

VofaReceive_PID PID_Mode[2];			/* 速度环/位置环 */


/* 通过Vofa+调参,用上位机上无线通信 */


/* 进入UART1接受中断 */

/* 参数数据包接收部分 */

/*
		速度环:Sp_kp  Sp_ki  Sp_kd
		位置环:Po_kp  Po_ki  Po_kd
		
		启动位:Start

*/

/*  	1.PID参数部分
			->
			#P1=12.123！   	#P为帧头，P1_x(0<=x<=2)为是改变谁的标志位， =是数据收集标志位

			12.123是数据本身  ！是帧尾	
		
		
			2.启动/停止部分
			->
			@Flag=1/2!			#@为帧头，Flag为该模式的标志字符部分，=是标志收集标志位
			
			1/2是标志位本身  ！是帧尾
*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
    if(huart->Instance == USART1)
    {
				static uint8_t PID_RxState=0;  		/* PID参数接收标志位 */
				static uint8_t On_or_off_RxState=0;		/* 启动停止位接收标志位 */
				static uint8_t PID_pRxPacket=0;			/* 接收数据位数(数组元素索引) */
				static uint8_t On_or_off_pRxPacket=0;			/* 接收数据位数(数组元素索引) */
			
        HAL_UART_Receive_IT(&huart1,&Rx_Byte,1);    /* 接受上位机传来的一个字节 */
				
				/* PID参数接收部分 */
				if(PID_RxState==0 && Rx_Byte==0x23)			/* 接收"#" */
				{
						PID_RxState=1;
				}
				else if(PID_RxState==1 && Rx_Byte==0x50)			/* 接收"P" */
				{
						PID_RxState=2;
				}
				else if(PID_RxState==2)		/* 接收标志1(_前边) */
				{
						id_Flag_1=Rx_Byte-48;
						PID_RxState=3;
				}
				else if(PID_RxState==3 && Rx_Byte==0x5F)			/* 接收"_" */
				{
						PID_RxState=4;
				}
				else if(PID_RxState==4)		/* 接收标志1(_后边) */
				{
						id_Flag_2=Rx_Byte-48;
						PID_RxState=5;					
				}
				else if(PID_RxState==5 && Rx_Byte==0x3D)		/* 接收"=" */
				{
						PID_RxState=6;
				}
				else if(PID_RxState==6)			/* 此时开始接收参数数据 */
				{
						if(Rx_Byte==0x21)		/* 接收到"!"即帧尾-->作标志位清零处理 */
						{
								Data_BitNum=PID_pRxPacket;		/* 获取接收到的实际位数 */		
								PID_pRxPacket=0;				/* 清除索引方便下次进行接收数据 */
								PID_RxState=0;						/* 标志位清零 */
								PID_RxFlag=1;		/* 获取标志位置1 */
						}
						else			/* 持续接收数据 */
						{
								Get_Vofa_RxData[PID_pRxPacket++]=Rx_Byte;
						}
				}
				
				
				/* 启动停止位接收部分 */
				if(On_or_off_RxState==0 && Rx_Byte==0x40)		/* 接收"@" */	
				{
						On_or_off_RxState=1;
				}
				else if(On_or_off_RxState==1 && Rx_Byte==0x46)		/* 接收"F" */
				{
						On_or_off_RxState=2;
				}
				else if(On_or_off_RxState==2 && Rx_Byte==0x6C)			/* 接收"l" */
				{
						On_or_off_RxState=3;
				}
				else if(On_or_off_RxState==3 && Rx_Byte==0x61)					/* 接收"a" */
				{
						On_or_off_RxState=4;
				}
				else if(On_or_off_RxState==4 && Rx_Byte==0x67)					/* 接收"g" */
				{
						On_or_off_RxState=5;
				}
				else if(On_or_off_RxState==5 && Rx_Byte==0x3D)			/* 接收"=" */
				{
						On_or_off_RxState=6;
				}
				else if(On_or_off_RxState==6)			/* 开始接受标志位 */
				{
						if(Rx_Byte==0x21)			/* 判断接收到帧尾"!"后,结束本次启动停止位的获取 */
						{
								On_or_off_RxState=0;				/* 标志位清零 */
								On_or_off_RxFlag=1;				/* 获取标志位置1 */
								On_or_off_pRxPacket=0;
						}
						else
						{
								Get_On_or_off[On_or_off_pRxPacket++]=Rx_Byte-48;		/* 获取对应启动停止位(1/2) */
						}
				}
    }
}


/* 获取PID数据接收成功标志位 */
uint8_t Get_vofa_PID_Rxflag(void)
{
		if(PID_RxFlag==1)		/* 成功时 */
		{
				PID_RxFlag=0;		/* 清零处理 */
				return 1;
		}
		else return 0;
}

/* 获取启动停止位成功标志位 */
uint8_t Get_On_or_off_Rxflag(void)
{
		if(On_or_off_RxFlag==1)
		{
				On_or_off_RxFlag=0;			/* 清零处理 */
				return 1;
		}
		else return 0;
}


/* 获取接收标志1 */
uint8_t Get_id_Flag_1(void)
{
		uint8_t id_temp;
		id_temp=id_Flag_1;
		id_Flag_1=0;			/* 清零 */
		return id_temp;
}

/* 获取接收标志2 */
uint8_t Get_id_Flag_2(void)
{
		uint8_t id_temp;
		id_temp=id_Flag_2;
		id_Flag_1=0;			/* 清零 */
		return id_temp;
}

float Pow_invert(uint8_t X,uint8_t n) /* x除以n次10 */
{
  float result=X;
	while(n--)
	{
		result/=10;
	}
	return result;
}

//uint8_t Get_Vofa_RxData[5]={0x31,0x32,0x2E,0x31,0x33};//可以给数据包直接赋值直接调用一下换算程序，看是否输出为12.13
//Data_BitNum = 5//别忘记数据的长度也要设置
//然后直接在主程序就放  Printf("%f\n",RxPacket_Data_Handle());  Delay_ms(1000);就ok了


/* 数据包换算处理 */
float RxPacket_Data_Handle(void)
{
  float Data=0.0;
  uint8_t dot_Flag=0;	/* 小数点标志位，能区分小数点后或小数点前 0为小数点前，1为小数点后 */
  uint8_t dot_after_num=1;	/* 小数点后的第几位 */
  int8_t minus_Flag=1;				/* 负号标志位 -1为是负号 1为正号 */
  for(uint8_t i=0;i<Data_BitNum;i++)
  {
    if(Get_Vofa_RxData[i]==0x2D)		/* 如果第一位为负号 */
    {
      minus_Flag=-1;
      continue;		/* 跳过本次循环 */ 
    }
    if(dot_Flag==0)
    {
      if(Get_Vofa_RxData[i]==0x2E) /* 如果识别到小数点，则将dot_Flag置1 */
      {
        dot_Flag=1;
      }
      else  /* 还没遇到小数点前的运算 */
      {
        Data = Data*10 + Get_Vofa_RxData[i]-48;
      }
    }
    else  /* 遇到小数点后的运算 */
    {
      Data = Data + Pow_invert(Get_Vofa_RxData[i]-48,dot_after_num);
      dot_after_num++;
    }
  }
  return Data*minus_Flag; /* 将换算后的数据返回出来 这里乘上负号标志位 */

}




