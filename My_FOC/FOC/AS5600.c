#include "AS5600.h"
 
float angle_prev=0; 
int full_rotations=0; // full rotation tracking;


//发送单字节时序
void AS5600_Write_Reg(unsigned char reg, unsigned char value)
{
	HAL_I2C_Mem_Write(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}


//发送多字节时序
void AS5600_Write_Regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	HAL_I2C_Mem_Write(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, value, len, 50);
}


//IIC读多字节
void AS5600_Read_Reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	HAL_I2C_Mem_Read(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}



//得到弧度制的角度，范围在0-6.28
float GetAngle_Without_Track(void)
{   
	float angle_d;
	int16_t in_angle;
	uint8_t temp[2]={0};
	AS5600_Read_Reg( Angle_Hight_Register_Addr, temp, 2);
	in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	angle_d = (float)in_angle * (2.0f*PI) / 4096;
	return angle_d;
	//angle_d为弧度制，范围在0-6.28
}


float GetAngle(void)
{
    float val = GetAngle_Without_Track();
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > (0.8f*2.0f*PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}

/* 获取角度值 */
float Get_Current_Angle(void)
{
	float angle_d;
	int16_t in_angle;
	uint8_t temp[2]={0};
	AS5600_Read_Reg( Angle_Hight_Register_Addr, temp, 2);
	in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	angle_d = (float)in_angle * 360 / 4096;
	return angle_d;
}

