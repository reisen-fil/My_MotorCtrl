#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f1xx_hal.h"
#include "Mymath.h"
#include "i2c.h"

/*
注意:AS5600的地址0x36是指的是原始7位设备地址,而ST I2C库中的设备地址是指原始设备地址左移一位得到的设备地址
*/

#define Slave_Addr                0x36<<1		//设备从地址
#define Write_Bit                 0	   			//写标记
#define Read_Bit                  1    			//读标记
#define Angle_Hight_Register_Addr 0x0C 			//寄存器高位地址
#define Angle_Low_Register_Addr   0x0D 			//寄存器低位地址

extern float Now_Angle,Last_Angle,AngleSpeed;		/* 计算转速所需的两个相对角度 */

void AS5600_Read_Reg(unsigned char reg, unsigned char* buf, unsigned short len);
void AS5600_Write_Reg(unsigned char reg, unsigned char value);
float GetAngle_Without_Track(void);
float GetAngle(void);

float Get_Current_Angle(void);
float Get_CalibraAngle(float Angle_Offset);	
float Get_AngleSpeed(void);


#endif /* __BSP_AS5600_H */
