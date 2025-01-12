#ifndef __PMSM_FOC_H
#define __PMSM_FOC_H

#include "stm32f1xx_hal.h"
#include "AS5600.h"
#include "Mymath.h"
#include "tim.h"

#include "adc.h"

extern TIM_HandleTypeDef htim2;

/* 采样到的三相电压 */
struct Three_Phase 
{
	float Ua,Ub,Uc;
	int Pwm_Ua,Pwm_Ub,Pwm_Uc;
};

/* DQ轴坐标系结构体 */
struct DQ_Aix
{
    /* 转矩分量与励磁分量 */
		float V_d;	    //d-q坐标系，d轴电压分量
		float V_q;	    //d-q坐标系，q轴电压分量
		float theta;	//转子电气角度
};

/* α-β轴坐标系结构体 */
struct AlphaBeta_Aix
{
		float V_Alpha;	    //α-β坐标系，α轴电压分量
		float V_Beta;		//α-β坐标系，β轴电压分量
};

/* SVPWM部分 */
struct SVPWM_Control
{
		uint8_t sector;	        //扇区
		float X;				//SVPWM算法中间变量
		float Y;				//SVPWM算法中间变量
		float Z;				//SVPWM算法中间变量
		float T_First;	        //SVPWM算法中间变量
		float T_Second;	        //SVPWM算法中间变量
		float T0;		 		//SVPWM算法中间变量
		float Ta;				//SVPWM算法中间变量
		float Tb;				//SVPWM算法中间变量
		float Tc;				//SVPWM算法中间变量
};

/* 三相占空比控制结构体 */
struct SVPWM_Duty
{	
		float Ta;			//A相占空比PWM寄存器的值
		float Tb;			//B相占空比PWM寄存器的值
		float Tc;			//C相占空比PWM寄存器的值
};

/* pid参数 */
typedef struct 
{
	float Kp,Ki,Kd;
	float target,erro,erro_last,now,i_sum,out;
	
}Str_pid;

extern Str_pid Speed_pid;    /* 速度环pid */
extern Str_pid Position_pid;   /* 位置式pid */
extern uint32_t MyADC_Value[2];

void Open_PMSM_FOC(void);

#endif

