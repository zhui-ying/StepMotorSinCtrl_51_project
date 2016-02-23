#ifndef _CONFIG_H_
#define _CONFIG_H_
//初始条件下，认为角度位于0°的位置，即步进电机的推动物位于轨道中间
//按键期间不要旋转码盘
#define ROLLBACK_ENABLE  0//是否支持反转 1 可以  0 不可以

//y=A*sin(n*x/N)
//y 输出长度
//A 幅值
//n 输入的倍频值
//N 内部分频系数
#define DIVIDE_FREQ  10//N 分频系数  
#define STEP_AMP 2000 //A 步进距离幅值，单位 mm， 步进电机每转一度，导轨距离变动约1mm
#define MAX_INPUT 10 //n 可输入的最大值调节,该值调节越大，对应的码盘转速要求越低

//步进电机设置，参考步进电机驱动器文档，每一项有0或1可选
#define DCY1 0
#define DCY2 1//50%衰减模式
#define TQ1 1
#define TQ2 0//75%力矩
	
#endif
