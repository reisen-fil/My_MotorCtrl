#include "PMSM_FOC.h"

uint32_t MyADC_Value[2];

/**********************************************************************************************************
Park变换，输入电角度、Ualpha和Ubeta，经过Park变换得到Uq、Ud
**********************************************************************************************************/ 
void Park_transit(struct DQ_Aix* Volt_QD,struct AlphaBeta_Aix* Volt_AB)       
{ 
    // float angle_el = Get_Current_Angle() * POLE_PAIR_NUM;       /* 根据极对数计算电角度 */

    Volt_QD->V_q = -Volt_AB->V_Alpha*sin(Volt_QD->theta) + Volt_AB->V_Beta*cos(Volt_QD->theta);       /* Park变换 */
    Volt_QD->V_d = Volt_AB->V_Alpha*cos(Volt_QD->theta) + Volt_AB->V_Beta*sin(Volt_QD->theta);
}

/**********************************************************************************************************
反park变换，输入Uq、Ud得到Ualpha、Ubeta
**********************************************************************************************************/ 
void Rev_Park(struct DQ_Aix* Volt_QD,struct AlphaBeta_Aix* Volt_AB)
{
	// Volt_QD->theta = Get_Current_Angle() * POLE_PAIR_NUM;       /* 根据极对数计算电角度 */

    Volt_AB->V_Alpha = Volt_QD->V_d*cos(Volt_QD->theta) - Volt_QD->V_q*sin(Volt_QD->theta);
    Volt_AB->V_Beta = Volt_QD->V_d*sin(Volt_QD->theta) + Volt_QD->V_q*cos(Volt_QD->theta);	

}

/**********************************************************************************************************
clark变换，输入Ua、Ub、Uc得到Ualpha、Ubeta
**********************************************************************************************************/ 
void clark_transit(struct Three_Phase* Volt,struct AlphaBeta_Aix* Volt_AB) 
{

    Volt_AB->V_Alpha = DIV_2_3*(Volt->Ua - DIV_1_2*Volt->Ub - DIV_1_2*Volt->Uc);   /* 做等幅值变换 */
    Volt_AB->V_Beta = DIV_2_3*(Sqrt_3_div_2*Volt->Ub + Sqrt_3_div_2*Volt->Uc);

}

/**********************************************************************************************************
反clark变换，输入Ualpha、Ubeta得到Ua、Ub、Uc
**********************************************************************************************************/ 
void Rev_clark(struct Three_Phase* Volt,struct AlphaBeta_Aix* Volt_AB) 
{

    Volt->Ua = Volt_AB->V_Alpha;        /* 依据Ia+Ib+Ic=0 */
    Volt->Ub = -DIV_1_2*Volt_AB->V_Alpha + Sqrt_3_div_2*Volt_AB->V_Beta;
    Volt->Uc = -DIV_1_2*Volt_AB->V_Alpha - Sqrt_3_div_2*Volt_AB->V_Beta;

}

/* 当前扇区判断/扇区计算 */
void CalculateSector(struct AlphaBeta_Aix ab, struct SVPWM_Control* ptr)
{
		int8_t A = 0;
		int8_t B = 0;
		int8_t C = 0;
		int8_t N = 0;
		
        /* 六个扇区的三个判断条件 */
		float VA = ab.V_Beta;
		float VB = ab.V_Alpha * Sqrt_3_div_2 - DIV_1_2 * ab.V_Beta;
		float VC = 0 - ab.V_Alpha * Sqrt_3_div_2 - DIV_1_2 * ab.V_Beta;
	
        /* 对X、Y、Z三个参量式赋值 */
		ptr->X = SVPWM_K * ab.V_Beta;
		ptr->Y = SVPWM_K * (ab.V_Alpha * Sqrt_3_div_2 + DIV_1_2 * ab.V_Beta);
		ptr->Z = SVPWM_K * (0 - ab.V_Alpha * Sqrt_3_div_2 + DIV_1_2 * ab.V_Beta);
	
		if(VA > 0)
		{
				A = 1;
		}
		else
		{
				A = 0;
		}
		
		if(VB > 0)
		{
				B = 1;
		}
		else
		{
				B = 0;
		}
		
		if(VC > 0)
		{
				C = 1;
		}
		else
		{
				C = 0;
		}
		
		N = 4*C + 2*B + A;
		
		switch(N)
		{
			case 3:
				ptr->sector = 1;
				break;
			case 1:
				ptr->sector = 2;
				break;
			case 5:
				ptr->sector = 3;
				break;
			case 4:
				ptr->sector = 4;
				break;
			case 6:
				ptr->sector = 5;
				break;
			case 2:
				ptr->sector = 6;
				break;
		}	
}

/* 计算出对应扇区占空比 */
void CalulateDutyCycle(struct SVPWM_Control* ctrl,struct SVPWM_Duty* ptr)
{
        /* 对X、Y、Z三个参量式 */
		switch(ctrl->sector)
		{
			case 2:
				ctrl->T_First = ctrl->Z;
				ctrl->T_Second = ctrl->Y;
				break;
			case 6:
				ctrl->T_First = ctrl->Y;
				ctrl->T_Second = 0 - ctrl->X;
				break;
			case 1:
				ctrl->T_First = 0 - ctrl->Z;
				ctrl->T_Second = ctrl->X;
				break;
			case 4:
				ctrl->T_First = 0 - ctrl->X;
				ctrl->T_Second = ctrl->Z;
				break;
			case 3:
				ctrl->T_First = ctrl->X;
				ctrl->T_Second = 0 - ctrl->Y;
				break;
			case 5:
				ctrl->T_First = 0 - ctrl->Y;
				ctrl->T_Second = 0 - ctrl->Z;
				break;
		}
		
		/**/
		if((ctrl->T_First + ctrl->T_Second) > Ts)
		{
				ctrl->T_First  = ctrl->T_First  * Ts / (ctrl->T_First + ctrl->T_Second);
				ctrl->T_Second = ctrl->T_Second  * Ts / (ctrl->T_First + ctrl->T_Second);
		}			
		
		ctrl->T0 = (Ts - ctrl->T_First - ctrl->T_Second) / 2;
		
		ctrl->Ta = ctrl->T0 / 2;
		ctrl->Tb = ctrl->Ta + ctrl->T_First / 2;
		ctrl->Tc = ctrl->Tb + ctrl->T_Second / 2;
		
		/* 判断对应扇区赋值三路占空比 */
		switch(ctrl->sector)
		{
			case 2:
				ptr->Ta = ctrl->Tb;
				ptr->Tb = ctrl->Ta;
				ptr->Tc = ctrl->Tc;
				break;
			case 6:
				ptr->Ta = ctrl->Ta;
				ptr->Tb = ctrl->Tc;
				ptr->Tc = ctrl->Tb;
				break;
			case 1:
				ptr->Ta = ctrl->Ta;
				ptr->Tb = ctrl->Tb;
				ptr->Tc = ctrl->Tc;
				break;
			case 4:
				ptr->Ta = ctrl->Tc;
				ptr->Tb = ctrl->Tb;
				ptr->Tc = ctrl->Ta;
				break;
			case 3:
				ptr->Ta = ctrl->Tc;
				ptr->Tb = ctrl->Ta;
				ptr->Tc = ctrl->Tb;
				break;
			case 5:
				ptr->Ta = ctrl->Tb;
				ptr->Tb = ctrl->Tc;
				ptr->Tc = ctrl->Ta;
				break;
		}
		
}

struct DQ_Aix test_DQ;
struct AlphaBeta_Aix test_AB;
struct SVPWM_Control test_ctrl;
struct SVPWM_Duty test_Duty;

/* 开环FOC-实现120r/min的转速 */
void Open_PMSM_FOC(void)
{
	test_DQ.theta+=(0.1f)*(PI/180.0);		/* 步距为0.1度 */
	
	test_DQ.V_q=3.5f;		/* 力矩电流 */

	Rev_Park(&test_DQ,&test_AB);			/* 反park变换获取静止坐标系中的Ia、Ib */
	CalculateSector(test_AB,&test_ctrl);		/* 扇区计算及扇区判断 */
	CalulateDutyCycle(&test_ctrl,&test_Duty);		/* 对应扇区的占空比计算 */

	TIM2->CCR1 = test_Duty.Ta;		/* 占空比设置 */
	TIM2->CCR2 = test_Duty.Tb;
	TIM2->CCR3 = test_Duty.Tc;
	
	if(test_DQ.theta>=2*PI) test_DQ.theta=0;		/* 转满一圈后清零 */
}

/* 角度闭环 */
void FOC_KeepAngle()
{
	test_DQ.theta+=(0.1f)*(PI/180.0);		/* 步距为0.1度 */
	
	test_DQ.V_q=3.5f;		/* 力矩电流 */

	Rev_Park(&test_DQ,&test_AB);			/* 反park变换获取静止坐标系中的Ia、Ib */
	CalculateSector(test_AB,&test_ctrl);		/* 扇区计算及扇区判断 */
	CalulateDutyCycle(&test_ctrl,&test_Duty);		/* 对应扇区的占空比计算 */

	TIM2->CCR1 = test_Duty.Ta;		/* 占空比设置 */
	TIM2->CCR2 = test_Duty.Tb;
TIM2->CCR3 = test_Duty.Tc;
	
	if(test_DQ.theta>=2*PI) test_DQ.theta=0;		/* 转满一圈后清零 */
}


/* 对实现120r/min转速下-->19.841us进入一次中断进行计算 */

/* 角度闭环(100ms) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		Open_PMSM_FOC();
		// HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&MyADC_Value,2);		/* 四路ADC通道 */
	}

}

// /* 电机速度环(无电流闭环) */
// void FOC_SpeedPID(struct Str_pid*po_pid)
// {
// 	po_pid->erro = po_pid->Target-pid->now;		/* 当前误差 */
// 	po_pid->out+ = po_pid->kp*(po_pid->erro-po_pid->erro_last) + po_pid->ki*po_pid->erro;		/* 输出 */
// 	po_pid->erro_last = po_pid->erro;		/* 前一次误差 */
// }


// /* 电机位置环(无电流闭环) */
// void FOC_PositionPID(struct Str_pid*sp_pid)
// {
// 	sp_pid->erro = sp_pid->Target-sp_pid->now;		/* 当前误差 */ 
// 	sp_pid->i_sum += sp_pid->erro;
// 	sp_pid->out = sp_pid->kp*sp_pid->erro + sp_pid->ki*sp_pid->i_sum + sp_pid->kp*(sp_pid->erro-sp_pid->erro_last);
// 	sp_pid->erro_last = sp_pid->erro;		/* 获取前次误差 */
// }



