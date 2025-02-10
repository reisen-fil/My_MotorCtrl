#ifndef __PMSM_FOC_H
#define __PMSM_FOC_H

#include "stm32f1xx_hal.h"
#include "AS5600.h"
#include "Mymath.h"
#include "tim.h"

#include "adc.h"
#include "main.h"
#include "test.h"

extern TIM_HandleTypeDef htim2;

extern uint16_t MyADC_Value[2];
extern float Current_theta;
extern float test_theta,test_speed;

extern struct Three_Phase Current_abc;		/* 实际的电机三相电流 */
extern struct AlphaBeta_Aix Current_AB;		/* 实际的AB轴电流 */
extern struct DQ_Aix Current_DQ;		/* 实际的力矩电流 */

/* 采样到的三相电压 */
struct Three_Phase 
{
	float Ia,Ib,Ic;
};

/* DQ轴坐标系结构体 */
struct DQ_Aix
{
    /* 转矩分量与励磁分量 */
		float I_d;	    //d-q坐标系，d轴电压分量
		float I_q;	    //d-q坐标系，q轴电压分量
		float theta;	//转子电气角度
};

/* α-β轴坐标系结构体 */
struct AlphaBeta_Aix
{
		float I_Alpha;	    //α-β坐标系，α轴电压分量
		float I_Beta;		//α-β坐标系，β轴电压分量
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

void Open_PMSM_FOC(void);
void Check_DriftOffsets(void);
void Set_ZeroAngle(void);

void FOC_SVPWM(float FOC_Iq,float FOC_Id);	/* d轴电流 */

#endif

