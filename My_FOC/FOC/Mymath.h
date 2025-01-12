#ifndef __MYMATH_H
#define __MYMATH_H

#include "stm32f1xx_hal.h"
#include "math.h"

#define PI		(3.141526535f)        //Pi的数学的定义
#define DIV_1_2	(0.5f)                //1除2
#define DIV_2_3	(0.66666f)            //2除3
#define Sqrt_3_div_2	(0.86602f)    //根号3除2
#define Vdc	(12.0f)                    //假设的VBus，这个根据自己的代码修改(12/2)
#define Ts	(1500*2)	              //周期，PWM计数器        
#define SVPWM_K (1.732f * Ts / Vdc)   //这个是SVPWM的算法公式
#define POLE_PAIR_NUM 7         /* 电机极对数 */


#endif 
