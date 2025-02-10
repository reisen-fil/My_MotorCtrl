#ifndef __MYMATH_H
#define __MYMATH_H

#include "stm32f1xx_hal.h"
#include "math.h"
#include "Mymath.h"

#define PI		(3.141526535f)        //Pi的数学的定义
#define DIV_1_2	(0.5f)                //1除2
#define DIV_2_3	(0.66666f)            //2除3
#define Sqrt_3_div_2	(0.86602f)    //根号3除2
#define Sqrt_3_div_3    (0.57733f)    //根号3除3
#define Vdc	(12.0f)                    //假设的VBus，这个根据自己的代码修改(12/2)
#define Ts	(1500*2)	              //周期，PWM计数器        
#define SVPWM_K (1.732f * Ts / Vdc)   //这个是SVPWM的算法公式
#define POLE_PAIR_NUM 7         /* 电机极对数 */

#define Ref  0.01f			/* 采样电阻阻值 */
#define Gain  50            /* 电流采样运放增益 */

#define AD_to_Current   8.14f       /* 采样后算出的电流值与实际值之间的倍数 */
#define AD_Offset       0      /* 采样电流偏移量补偿 */

#endif 
