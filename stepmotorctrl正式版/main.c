//步进电机200 p/r 也就是步距角1.8°
//光电码盘 400r/r 精度角 0.9°
//步进电机脉冲频率 10KHz   1/8细分 

#include <reg52.h>
#include <intrins.h>
#include <stdio.h>
#include <stdarg.h>
#include "config.h"

//#define FOSC 11059200L //晶振设置，默认使用11.0592M Hz
//#define FOSC 12000000L //晶振设置，使用12M Hz
#define FOSC 48000000L //晶振设置，使用24M Hz
#define BAUD 9600
#define TIME_MS    63536//65336//(65536-FOSC/12/1000*0.05) //设定定时时间 0.05ms ,10000Hz 5r/s


//IO接口定义
//按键接口 低有效
sbit key_plus = P2^1;
sbit key_minus = P2^0;
//数码管接口 低有效
sbit we1 = P2^3;
sbit we2 = P2^2;
#define LED_PORT P0 
//步进电机配置IO
sbit motor_dcy1 = P1^0;
sbit motor_dcy2 = P1^1;//电流衰减模式选择
sbit motor_m1 = P1^2;
sbit motor_m2 = P1^3;//激励模式选择，选择是否分频
sbit motor_pro = P1^4;//保护反馈 input
sbit motor_m0 = P1^5;//初始状态反馈 input
sbit motor_tq1 = P1^6;
sbit motor_tq2 = P1^7;//输出力矩设置，电流值越大，力矩越强
sbit motor_step = P3^4;//步进脉冲输出   经过光耦反相
sbit motor_dir = P3^5;//控制电机转动方向  经过光耦反相
sbit motor_en = P3^6;//电机使能  经过光耦反相
//编码器IO
sbit coder_a = P3^2;
sbit coder_b = P3^3;

//全局变量定义
//LED显示字模 0-F 共阳模式
unsigned code table[]= {0Xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};
//正弦参数表0-2π，间隔0.9°  (sin(x)+1)*1000
unsigned int code sintable[401]={
1000,1016,1031,1047,1063,1078,1094,1110,1125,1141,1156,1172,1187,1203,1218,1233,1249,1264,1279,1294,1309,1324,1339,1353,1368,1383,1397,1412,1426,1440,1454,1468,1482,1495,1509,1522,1536,1549,1562,1575,
1588,1600,1613,1625,1637,1649,1661,1673,1685,1696,1707,1718,1729,1740,1750,1760,1771,1780,1790,1800,1809,1818,1827,1836,1844,1853,1861,1869,1876,1884,1891,1898,1905,1911,1918,1924,1930,1935,1941,1946,
1951,1956,1960,1965,1969,1972,1976,1979,1982,1985,1988,1990,1992,1994,1996,1997,1998,1999,2000,2000,2000,2000,2000,1999,1998,1997,1996,1994,1992,1990,1988,1985,1982,1979,1976,1972,1969,1965,1960,1956,
1951,1946,1941,1935,1930,1924,1918,1911,1905,1898,1891,1884,1876,1869,1861,1853,1844,1836,1827,1818,1809,1800,1790,1780,1771,1760,1750,1740,1729,1718,1707,1696,1685,1673,1661,1649,1637,1625,1613,1600,
1588,1575,1562,1549,1536,1522,1509,1495,1482,1468,1454,1440,1426,1412,1397,1383,1368,1353,1339,1324,1309,1294,1279,1264,1249,1233,1218,1203,1187,1172,1156,1141,1125,1110,1094,1078,1063,1047,1031,1016,
1000,984,969,953,937,922,906,890,875,859,844,828,813,797,782,767,751,736,721,706,691,676,661,647,632,617,603,588,574,560,546,532,518,505,491,478,464,451,438,425,
412,400,387,375,363,351,339,327,315,304,293,282,271,260,250,240,229,220,210,200,191,182,173,164,156,147,139,131,124,116,109,102,95,89,82,76,70,65,59,54,
49,44,40,35,31,28,24,21,18,15,12,10,8,6,4,3,2,1,0,0,0,0,0,1,2,3,4,6,8,10,12,15,18,21,24,28,31,35,40,44,
49,54,59,65,70,76,82,89,95,102,109,116,124,131,139,147,156,164,173,182,191,200,210,220,229,240,250,260,271,282,293,304,315,327,339,351,363,375,387,400,
412,425,438,451,464,478,491,505,518,532,546,560,574,588,603,617,632,647,661,676,691,706,721,736,751,767,782,797,813,828,844,859,875,890,906,922,937,953,969,984,
1000
};

char ratio_value=1;//0-MAX_INPUT
unsigned int motor_count=0;
unsigned int coder_count=0;//0 - 3600 每接收一个脉冲加9，计数到3600就归0

volatile unsigned int time1s_tick=0;
unsigned int t_flag=0;
volatile unsigned int position_set = STEP_AMP*4;//目标位置，处于中间位置, 单位 1mm  最大值16000
volatile unsigned int position_cur = STEP_AMP*4;//当前位置, 单位 1mm 最大值16000
unsigned int code_count_set=0;//计算之后的角度

volatile unsigned char time_h = 65336/256;
volatile unsigned char time_l = 65336%256;//10kHz
volatile unsigned int step_period = 63536;

char putchar(char ch)
{ 
	/* Place your implementation of fputc here */ 
	SBUF=(unsigned char)ch; //将接收到的数据放入到发送寄存器
	while(!TI);		  //等待发送数据完成
	TI=0;		 //清除发送完成标志位	
	return ch;
}

void Delayms(unsigned int ms)
{
	unsigned int i,j;
	for(i=0;i<ms;i++)
	#if FOSC == 11059200L
	for(j=0;j<114;j++);
	#elif FOSC == 12000000L
	for(j=0;j<123;j++);
	#elif FOSC == 24000000L
	for(j=0;j<249;j++);
  #elif FOSC == 48000000L
	for(j=0;j<500;j++);
	#else
	for(j=0;j<114;j++);
	#endif
}

void Timer0Init()
{	
	TMOD |=  0x01; //设置定时器0工作方式为1
	TH0=TIME_MS/256;
	TL0=TIME_MS%256;
	ET0=1; //开启定时器0中断
	TR0=1;	//开启定时器	
	EA=1;  //打开总中断
}

//外部中断0初始化，接收编码器脉冲
void Exit0Init()
{
	EX0 = 1;	//使能 INT1 外部中断
	IT0 = 1;	// 触发方式为脉冲负边沿触发
	EA = 1;//总中断
}  

void UsartConfiguration()
{
	SCON = 0X50;			//设置为工作方式1	10位异步收发器
	TMOD |= 0x20; //设置计数器工作方式2  8位自动重装计数器	
	PCON = 0X80;//波特率加倍	
	TH1 = 256 -(FOSC/12/32/(BAUD/2)); //计算溢出率
	TL1 = 256 -(FOSC/12/32/(BAUD/2));
	TR1 = 1; //打开定时器	
	EA = 1;//打开总中断
}

void SystemInit()
{
	key_plus = 1;
	key_minus = 1;
	LED_PORT = 0xff;
	we1 = 1;
	we2 = 1;
	
	//步进电机设置
	motor_dcy1 = DCY1;
	motor_dcy2 = DCY2;//50%衰减模式
	motor_m1 = 1;
	motor_m2 = 1;//1/8细分
	motor_tq1 = TQ1;
	motor_tq2 = TQ2;//75%力矩
	motor_dir = 0;//方向
	motor_en = 0;//使能
}

void KeyScan() 
{
	if(key_plus==0)  //判断是否按下键盘
	{
		Delayms(5); //延时,软件去干扰
		if(key_plus==0)   //确认按键按下
		{
			ratio_value++;     //按键计数加1
			if(ratio_value == (MAX_INPUT+1))
			{
				ratio_value = MAX_INPUT;   
			}
			printf("ratio:%d\r\n",(int)ratio_value);
			
			code_count_set = (coder_count*ratio_value+5)/DIVIDE_FREQ;//四舍五入
			position_set = sintable[code_count_set%400] * (STEP_AMP/1000);//求目标位置
			position_set = position_set<<2;//换算成步进电机需要的脉冲数，4p/度 = 4p/mm
			position_cur = position_set;
			
		}
		while(key_plus==0);//按键锁定,每按一次count只加1.
	}
	
	if(key_minus==0)  //判断是否按下键盘
	{
		Delayms(5); //延时,软件去干扰
		if(key_minus==0)   //确认按键按下
		{
			ratio_value--;     //按键计数减1
			if(ratio_value == -1) 
			{
				ratio_value = 0;   
			}
			printf("ratio:%d\r\n",(int)ratio_value);
			
			code_count_set = (coder_count*ratio_value+5)/DIVIDE_FREQ;//四舍五入
			position_set = sintable[code_count_set%400] * (STEP_AMP/1000);//求目标位置
			position_set = position_set<<2;//换算成步进电机需要的脉冲数，4p/度 = 4p/mm
			position_cur = position_set;
		}
		while(key_minus==0);//按键锁定,每按一次count只加1.
	}	
}

void Display(char value)
{
	char ge,shi;
	ge = value%10;
	shi = value%100/10;
	
	we1 = 0;
	LED_PORT = table[shi];
	Delayms(2);
	LED_PORT = 0xff;
	we1 = 1;
	
	we2 = 0;
	LED_PORT = table[ge];
	Delayms(2);
	LED_PORT = 0xff;
	we2 = 1;
}

/*******************************************************************************
* 函 数 名 ：main
* 函数功能 ：主函数
* 输    入 ：无
* 输    出 ：无
*******************************************************************************/
void main()
{
	//unsigned int diff_position;
//	unsigned int angle;
	UsartConfiguration();
	printf("system begin\r\n");
	Timer0Init();
	Exit0Init();
	SystemInit();	
	//Delayms(1000);

	while(1)
	{
		KeyScan();
		Display(ratio_value);
		if(time1s_tick>= 100)
		{
			time1s_tick = 0;
			printf("%u	%u\r\n",coder_count,position_cur);//position_set
		}
	
		if((position_cur < 100) || (position_cur > 15900))//接近低谷或峰值要反转时减速 1k
		{
			time_h = 63536/256;
			time_l = 63536%256;			
		}else	if((position_cur < 400) || (position_cur > 15600))//接近低谷或峰值要反转时减速 5k
		{
			time_h = 64436/256;
			time_l = 64436%256;			
		}else  //  10k
		{
			time_h = 65336/256;
			time_l = 65336%256;				
		}
	}	
}

//500us
void Timer0Int() interrupt 1
{
	TH0=time_h;//TIME_MS/256
	TL0=time_l;//TIME_MS%256

	time1s_tick++;
	if(position_set > position_cur)//正转
	{
		motor_dir = 0;
		motor_step = !motor_step;
		position_cur++;//+0.25mm
	}
	 else if(position_set < position_cur)//反转
	{
		motor_dir = 1;
		motor_step = !motor_step;
		position_cur--;//-0.25mm	
	}
}

void Exit0Int() interrupt 0 
{
	if(coder_b == 1)//正转
	{
		if(coder_count == 4000) 
		{
			coder_count = 0;
		}
		coder_count++;	
	}else
	{
		#if ROLLBACK_ENABLE == 1
		if(coder_count == 0) 
		{	
			coder_count = 4000;
		}
		coder_count--;//反转	
		#else
		return;//禁止反转
		#endif
	}
		code_count_set = (coder_count*ratio_value+5)/DIVIDE_FREQ;//四舍五入
		position_set = sintable[code_count_set%400] * (STEP_AMP/1000);//求目标位置
		position_set = position_set<<2;//换算成步进电机需要的脉冲数，4p/度 = 4p/mm
}

