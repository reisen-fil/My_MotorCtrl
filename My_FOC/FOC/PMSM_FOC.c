#include "PMSM_FOC.h"

uint16_t MyADC_Value[2];
float Amp_Offset[2];		/* 采样电压偏置 */

float Iq_ref,Id_ref;		/* 电流环反馈值 */
float Current_theta;
float ZeroAngle_ref;    /* 零电角度参考值 */

float test_theta,test_speed;		/* 测试变量 */

Str_pid Speed_pid;    /* 速度环pid */
Str_pid Position_pid;   /* 位置式pid */

float Lowpassfilter(float Tf, float x);

/**********************************************************************************************************
Park变换，输入电角度、Ualpha和Ubeta，经过Park变换得到Uq、Ud
**********************************************************************************************************/ 
void Park_transit(struct DQ_Aix* I_QD,struct AlphaBeta_Aix* I_AB)       
{ 
    // float angle_el = Get_Current_Angle() * POLE_PAIR_NUM;       /* 根据极对数计算电角度 */

    I_QD->I_q = -I_AB->I_Alpha*sin(I_QD->theta) + I_AB->I_Beta*cos(I_QD->theta);       /* Park变换 */
    I_QD->I_d = I_AB->I_Alpha*cos(I_QD->theta) + I_AB->I_Beta*sin(I_QD->theta);
}

/**********************************************************************************************************
反park变换，输入Uq、Ud得到Ualpha、Ubeta
**********************************************************************************************************/ 
void Rev_Park(struct DQ_Aix* I_QD,struct AlphaBeta_Aix* I_AB)
{
	// I_QD->theta = Get_Current_Angle() * POLE_PAIR_NUM;       /* 根据极对数计算电角度 */
	// I_QD->theta = test_DQ.theta;

    I_AB->I_Alpha = I_QD->I_d*cos(I_QD->theta) - I_QD->I_q*sin(I_QD->theta);
    I_AB->I_Beta = I_QD->I_d*sin(I_QD->theta) + I_QD->I_q*cos(I_QD->theta);	

}

/**********************************************************************************************************
clark变换，输入Ua、Ub、Uc得到Ualpha、Ubeta
**********************************************************************************************************/ 
void clark_transit(struct Three_Phase* I_abc,struct AlphaBeta_Aix* I_AB) 
{

    I_AB->I_Alpha = DIV_2_3*(I_abc->Ia - DIV_1_2*I_abc->Ib - DIV_1_2*I_abc->Ic);   /* 做等幅值变换 */
    I_AB->I_Beta = DIV_2_3*(Sqrt_3_div_2*I_abc->Ib - Sqrt_3_div_2*I_abc->Ic);

}

/**********************************************************************************************************
反clark变换，输入Ualpha、Ubeta得到Ua、Ub、Uc
**********************************************************************************************************/ 
void Rev_clark(struct Three_Phase* I_abc,struct AlphaBeta_Aix* I_AB) 
{

    I_abc->Ia = I_AB->I_Alpha;        /* 依据Ia+Ib+Ic=0 */
    I_abc->Ib = -DIV_1_2*I_AB->I_Alpha + Sqrt_3_div_2*I_AB->I_Beta;
    I_abc->Ic = -DIV_1_2*I_AB->I_Alpha - Sqrt_3_div_2*I_AB->I_Beta;

}

/* 当前扇区判断/扇区计算 */
void CalculateSector(struct AlphaBeta_Aix ab, struct SVPWM_Control* ptr)
{
		int8_t A = 0;
		int8_t B = 0;
		int8_t C = 0;
		int8_t N = 0;
		
        /* 六个扇区的三个判断条件 */
		float VA = ab.I_Beta;
		float VB = ab.I_Alpha * Sqrt_3_div_2 - DIV_1_2 * ab.I_Beta;
		float VC = 0 - ab.I_Alpha * Sqrt_3_div_2 - DIV_1_2 * ab.I_Beta;
	
        /* 对X、Y、Z三个参量式赋值 */
		ptr->X = SVPWM_K * ab.I_Beta;
		ptr->Y = SVPWM_K * (ab.I_Alpha * Sqrt_3_div_2 + DIV_1_2 * ab.I_Beta);
		ptr->Z = SVPWM_K * (0 - ab.I_Alpha * Sqrt_3_div_2 + DIV_1_2 * ab.I_Beta);
	
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
        /* 对X、Y、Z三个参量式,在扇区判断的基础上进行扇区计算值的赋值 */
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
		
		/* 当大于Ts本身时,对T_First和T_Second的大小进行缩放 */
		if((ctrl->T_First + ctrl->T_Second) > Ts)
		{
				ctrl->T_First  = ctrl->T_First  * Ts / (ctrl->T_First + ctrl->T_Second);
				ctrl->T_Second = ctrl->T_Second  * Ts / (ctrl->T_First + ctrl->T_Second);
		}			
		
		ctrl->T0 = (Ts - ctrl->T_First - ctrl->T_Second) / 2;
		
		/* 按照PWM中心对齐模式2、PWM计数模式1来设置三路逆变器PWM的大小 */
		ctrl->Tc = ctrl->T0 / 2;
		ctrl->Tb = ctrl->Tc + ctrl->T_Second / 2;
		ctrl->Ta = ctrl->Tb + ctrl->T_First / 2;

		/* 按照PWM中心对齐模式1、PWM计数模式2来设置三路逆变器PWM的大小 */
		// ctrl->Ta = ctrl->T0 / 2;
		// ctrl->Tb = ctrl->Ta + ctrl->T_First / 2;
		// ctrl->Tc = ctrl->Tb + ctrl->T_Second / 2;
		
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

struct Three_Phase Current_abc;		/* 实际的电机三相电流 */

struct DQ_Aix test_DQ;
struct DQ_Aix Current_DQ;		/* 实际的力矩电流 */

struct AlphaBeta_Aix test_AB;
struct AlphaBeta_Aix Current_AB;		/* 实际的AB轴电流 */

struct SVPWM_Control test_ctrl;
struct SVPWM_Duty test_Duty;

/* SVPWM算法 */
void FOC_SVPWM(float FOC_Iq,float FOC_Id)
{
	test_DQ.I_q=FOC_Iq;		/* q轴力矩电流 */
	test_DQ.I_d=FOC_Id;		/* d轴转轴部分 */

	Rev_Park(&test_DQ,&test_AB);			/* 反park变换获取静止坐标系中的Ia、Ib */
	CalculateSector(test_AB,&test_ctrl);		/* 扇区计算及扇区判断 */
	CalulateDutyCycle(&test_ctrl,&test_Duty);		/* 对应扇区的占空比计算 */

	TIM2->CCR1 = test_Duty.Ta;		/* 占空比设置 */
	TIM2->CCR2 = test_Duty.Tb;
	TIM2->CCR3 = test_Duty.Tc;	
}

/* 开环FOC-实现对应转速 */

/* 帕克逆变换的旋转电角度为正的时候正转,为负的时候反转 */

/* 角度人为自增的FOC控制 */
void Open_PMSM_FOC(void)
{
	test_DQ.theta+=(2.0f)*(PI/180.0);		/* 步距为1度 */
	if(test_DQ.theta>=2*PI) test_DQ.theta=0;		/* 转满一圈后清零 */

	// test_DQ.theta+=(-1.0f)*(PI/180.0);		/* 步距为1度 */
	// if(test_DQ.theta<=-2*PI) test_DQ.theta=0;		/* 转满一圈后清零 */

	// Current_DQ.theta = test_DQ.theta;	/* 电角度更新 */	

	FOC_SVPWM(1.5f,0.0f);
}

/* 获取的电角度为编码器角度转换的FOC控制 */
void FOC_Mode(float Iq,float Id)
{
	test_DQ.theta = Get_CalibraAngle(120.0f)*PI/180;	

	FOC_SVPWM(Iq,Id);
}

/* 零电角度校准 */

/* 在该情况下，电角度在大约120度时近乎与编码器实现零点对齐(在编码器位置以及计数方向变
化的时候应重新获取零点对应电角度值) */

void Set_ZeroAngle(void)
{
	/* 给定适当电流实现零角度初始定位 */
	test_DQ.theta=120*(PI/180.0);	

	FOC_SVPWM(0.0f,1.0f);	/* d轴电流 */

	/* 速度闭环时,计算实际转速的双角度校准 */
	Now_Angle = GetAngle();
	Last_Angle = Now_Angle;

	// ZeroAngle_ref = Get_Current_Angle();/* 返回初始定位时编码器测得的机械角度值(角度参考值) */
}

/* 获取编码器测得的变化的电角度值(返回弧度值) */
float Get_FOCAngle(void)
{
	float Current_Angle = (Get_Current_Angle() - ZeroAngle_ref)*POLE_PAIR_NUM*(PI/180.0);
	return Current_Angle;
}


/* 开环控制/闭环 */
/* 定时2ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		// Open_PMSM_FOC();

		test_speed = Speed_pid.now;
		// test_speed = Get_AngleSpeed();
		Speed_FOC_Ctrl();

		// I_Feedback_Sample();
		// Iq_FOCctrl();

		// test_theta = Get_CalibraAngle(120.0f);	
	}
}

/* 采样电流低通滤波器 */
float y = 0;
float Lowpassfilter_sim(float x)
{
	float out = 0.9*x + 0.1*y;
	y = x;
	return out;
}

uint32_t Last_Timesamp = 0.0;
float Last_y = 0.0;
float Lowpassfilter(float Tf, float x)
{
	float dt = 0.0;

	uint32_t Timesamp = SysTick->VAL;
	if(Timesamp < Last_Timesamp) dt = (float)(Last_Timesamp - Timesamp)/9*1e-6;
	else
		dt = (float)(0xFFFFFF - Timesamp + Last_Timesamp)/9*1e-6;

	if(dt<0.0||dt==0) dt = 0.0015f;
	else if(dt>0.05f)
	{
		Last_y = x;
		Last_Timesamp = Timesamp;
		return x;
	}
	float alpha = Tf / (Tf + dt);
	float y = alpha * Last_y + (1.0f - alpha) * x;
	
	Last_y = y;
	Last_Timesamp = Timesamp;
	return y;
}

/* 限幅部分 */
float LimitAmp(float Amp,float Amp_max,float Amp_min)
{
	if(Amp >= Amp_max) Amp=Amp_max;
	else if(Amp <= Amp_min) Amp=Amp_min;
	return Amp;
}

/* 电流反馈采样部分 */
void I_Feedback_Sample(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&MyADC_Value,sizeof(MyADC_Value)/sizeof(MyADC_Value[0]));		/* 两路ADC通道 */

	MyADC_Value[0] = MyADC_Value[0]+AD_Offset;
	MyADC_Value[1] = MyADC_Value[1]+AD_Offset;

	/* 采样电流计算 */
	Current_abc.Ia = (3.3*((float)MyADC_Value[0]/4096)-Amp_Offset[0])/Ref/Gain;
	Current_abc.Ib = (3.3*((float)MyADC_Value[1]/4096)-Amp_Offset[1])/Ref/Gain;
	Current_abc.Ic = -Current_abc.Ia-Current_abc.Ib;  /* 根据基尔霍夫电流定律得 */

	Current_DQ.theta = Get_CalibraAngle(120.0f)*PI/180;

	clark_transit(&Current_abc,&Current_AB);			/* clark变换 */
	Park_transit(&Current_DQ,&Current_AB);				/* Park变换 */

	Current_DQ.I_q = -AD_to_Current*Lowpassfilter(0.01,Current_DQ.I_q);		/* 低通滤波后的Iq电流 */
	// Current_DQ.I_d = Lowpassfilter(0.01,Current_DQ.I_d);		/* Id电流 */
}

/* 检测电流采样放大器偏置(抑制零飘) */
void Check_DriftOffsets(void)
{
	uint16_t detect_time = 1000;
	for(int i=0;i<detect_time;i++)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&MyADC_Value,sizeof(MyADC_Value)/sizeof(MyADC_Value[0]));		/* 两路ADC通道 */
		Amp_Offset[0] += 3.3*((float)MyADC_Value[0]/4096);
		Amp_Offset[1] += 3.3*((float)MyADC_Value[1]/4096);
	}

	Amp_Offset[0] = Amp_Offset[0]/detect_time;
	Amp_Offset[1] = Amp_Offset[1]/detect_time;
}


/* 电流环pid */
void Current_PID_Ctrl(float Now_Iq,float Target_Iq,float Now_Id,float Target_Id,float Kp,float Ki)
{
	/* Iq闭环部分 */
	float Last_Iq,Iq_Error,Iq_Bias,Last_Iq_Bias,Iq_Sum;
	Iq_Bias = Target_Iq - Now_Iq;
	Iq_Error = Iq_Bias - Last_Iq_Bias;
	
	Iq_Sum += Iq_Error;		/* Ki积分累积 */

	Iq_ref = Kp*Iq_Error + Ki*Iq_Sum;

	LimitAmp(Iq_ref,7.0f,-7.0f);	/* 积分限幅 */

	Last_Iq_Bias = Iq_Bias;

	/* Id闭环部分 */
	float Last_Id,Id_Error,Id_Bias,Last_Id_Bias,Id_Sum;
	Id_Bias = Target_Id - Now_Id;
	Id_Error = Id_Bias - Last_Id_Bias;
	
	Id_Sum += Id_Error;		/* Ki积分累积 */

	Id_ref = Kp*Id_Error + Ki*Id_Sum;

	LimitAmp(Id_ref,1.0f,-1.0f);	/* 积分限幅 */

	Last_Id_Bias = Id_Bias;	

	test_DQ.theta = Current_DQ.theta;

	// test_DQ.theta = Get_FOCAngle();
	// test_theta = Get_CalibraAngle(120.0f)*PI/180;

	FOC_SVPWM(Iq_ref,Id_ref);
}

/* FOC电流环闭环控制 */
void Iq_FOCctrl(void)
{
	I_Feedback_Sample();  /* q轴电流以及d轴电流采样 */

	Current_PID_Ctrl(Current_DQ.I_q,3.5f,Current_DQ.I_d,1.0f,0.5f,0.0f);		/* PID闭环控制 */
}

/* 速度环pid */
void Speed_PID_Ctrl(Str_pid* ctrl)
{
	ctrl->erro = ctrl->target-ctrl->now;

	ctrl->i_sum += ctrl->erro;
	ctrl->out = ctrl->Kp*(ctrl->erro-ctrl->erro_last)+ctrl->Ki*ctrl->i_sum;
	ctrl->out = LimitAmp(ctrl->out,3.0f,-3.0f);		/* 限幅部分 */

	ctrl->erro_last = ctrl->erro;
}

/* FOC速度环(无电流内环) */
void Speed_FOC_Ctrl(void)
{
	Speed_pid.now = Get_AngleSpeed();/* 计算电机实际角速度 */
	Speed_pid.target=12.5f;			/* 目标速度 */

	Speed_PID_Ctrl(&Speed_pid);		/* 闭环控制 */
	FOC_Mode(Speed_pid.out,0.0f);	/* FOC矢量控制 */
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



