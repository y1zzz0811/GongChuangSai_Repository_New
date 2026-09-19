#include "data.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "Servo.h"
#include "thread_rccu.h"
#include "SCServo.h"
#include "SLAVE_SteeringEngine_3CH/SLAVE_SteeringEngine_3CH.h"
typedef struct{
	int16_t Num1;//舵机ID
	int16_t Num2;
	int16_t Num3;
	int16_t Num4;
}Servo;
//int16_t Red_1 = 1912;
//int16_t Green_1 = 3293;
//int16_t Blue_1 = 582;


//int16_t Open_Claw = 800;
//int16_t Mid_Claw = 1100;
//int16_t Close_Claw = 1370;
int16_t Open_Claw = 2070;
int16_t Mid_Claw = 1700;
int16_t Close_Claw = 1550;
/* ------------------------------------------------------------*/
Servo bowl_1 		= {//1号碗
								773,
								2200,
								2172,//>>>>>2050
								};
Servo bowl_2 		= {//2号碗
								1014,
								2150,
								2165,//>>>>>2050
								};
Servo bowl_3 		= {//3号碗
								1251,
								2200,
								2164,//>>>>>2050
								};
/* ------------------------------------------------------------*/
								
							/*	3580,
								2680,
								1660*/
Servo Red 		= {//红一层
								3545,
								2700,
								1684
								};
Servo Green 	= {//绿一层
								3065,
								2520,
								1471
								};
Servo Blue 		= {//蓝一层
								2620,
								2685,
								1660
								};
/* ------------------------------------------------------------*/
Servo Red_Stacking 		= {//红码垛
								3521,
								2543,
								1981	
								};
Servo Green_Stacking 	= {//绿码垛
								3053,
								2297,
								1870
								};
Servo Blue_Stacking 	= {//蓝码垛
								2620,
								2536,
								1951
								};
/* ------------------------------------------------------------*/

		void Rest_action()//归位动作
		{
			WritePosEx( 2, 1835, 3250, 60);
			WritePosEx( 3, 2382, 3250, 60);
			My_mDelay(800);
		}
		void Rest_action_S2()//初始动作
		{
			WritePosEx(2, 2140, 3250, 60);		//2号舵机控制前后
			WritePosEx(3, 2280, 3250, 60);		//3号舵机控制上下
			HAL_Delay(200);
			WritePosEx(1, 1040, 2500,60);	
			WritePosEx(4, Mid_Claw, 3250, 60);		//4号舵机手爪
			HAL_Delay(500);
		}
		
		void QR_action()//识别二维码动作
		{
			WritePosEx( 1, 3108, 3250, 60);
			WritePosEx( 2, 1919, 3250, 60);
			WritePosEx( 3, 2373, 3250, 60);
			WritePosEx(4, Close_Claw, 3250, 60);		//4号舵机手爪
			My_mDelay(800);
		}
		
		void IdentifyColor_action()//识别原料颜色动作
		{
			WritePosEx(1, 3076, 2500,60);
			WritePosEx(2, 2098, 3250, 60);
			WritePosEx(3, 2386, 3250, 60);
			WritePosEx(4, Mid_Claw, 3250, 60);
			HAL_Delay(800);
		}
		void IdentifyCircle_action()//粗加工区识别圆环动作
		{
			WritePosEx(1, 3063, 2500,60);
			HAL_Delay(800);
			WritePosEx(2, 2155, 3250, 60);
			WritePosEx(3, 2386, 3250, 60);//2154
			WritePosEx(4, Mid_Claw, 3250, 60);
			HAL_Delay(800);
		}
		void Get_A()//抓取原料区物料
		{
			WritePosEx(4, Open_Claw, 4250, 120);
			HAL_Delay(200);
			WritePosEx(1, 3051, 2500,60);
			WritePosEx(2, 2307, 3250, 60);
			WritePosEx(3, 1962, 3250, 60);
			HAL_Delay(500);
			WritePosEx(4, Close_Claw,  4250, 120);//抓
			HAL_Delay(400);
			Rest_action();//归位
		}
		void Put_1()//放到1号碗（红）
		{
			WritePosEx(1, bowl_1.Num1, 2500,60);//转到1号碗方向
			HAL_Delay(1200);
			WritePosEx(2, bowl_1.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(3, bowl_1.Num3, 1897, 30);//向下伸
			HAL_Delay(300);
			WritePosEx(4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Put_2()//放到2号碗（绿）
		{
			WritePosEx(1, bowl_2.Num1, 2500,60);//转到2号碗方向
			HAL_Delay(1000);
			WritePosEx(2, bowl_2.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(3, bowl_2.Num3, 1897, 30);//向下伸
			HAL_Delay(300);
			WritePosEx(4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Put_3()//放到3号碗（蓝）
		{
			WritePosEx(1, bowl_3.Num1, 2500,60);//转到3号碗方向
			HAL_Delay(800);
			WritePosEx(2, bowl_3.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(3, bowl_3.Num3, 1897, 30);//向下伸	
			HAL_Delay(300);
			WritePosEx(4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Get_1()//抓1号碗物料（红）
		{
			WritePosEx(4, Open_Claw, 3250, 60);//放
			WritePosEx(1, bowl_1.Num1, 2500,60);//转到1号碗方向
			HAL_Delay(500);
			WritePosEx(2, bowl_1.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(3, bowl_1.Num3, 3250, 60);//向下伸
			HAL_Delay(400);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			WritePosEx(3, 2382, 1897, 30);//向上伸			
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Get_2()//抓2号碗物料（绿）
		{
			WritePosEx(4, Open_Claw, 3250, 60);//放
			WritePosEx(1, bowl_2.Num1, 2500,60);//转到2号碗方向
			HAL_Delay(800);
			WritePosEx(2, bowl_2.Num2, 3250, 60);//向前伸		
			HAL_Delay(800);
			WritePosEx(3, bowl_2.Num3, 3250, 60);//向下伸			
			HAL_Delay(300);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			WritePosEx(3, 2382, 1897, 30);//向上伸			
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Get_3()//抓3号碗物料（蓝）
		{
			WritePosEx(4, Open_Claw, 3250, 60);//放
			WritePosEx(1, bowl_3.Num1, 2500,60);//转到3号碗方向
			HAL_Delay(800);
			WritePosEx(2, bowl_3.Num2, 3250, 60);//向前伸		
			HAL_Delay(800);
			WritePosEx(3, bowl_3.Num3, 3250, 60);//向下伸		
			HAL_Delay(300);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			WritePosEx(3, 2382, 1897, 30);//向上伸
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Put_Red()//放红环——第一层
		{
			WritePosEx(1, Red.Num1, 2500,60);//转到红环方向
			HAL_Delay(1400);
			WritePosEx(2, Red.Num2, 3250, 60);
			WritePosEx(3, Red.Num3, 3250, 60);
			HAL_Delay(800);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(500);//300
			Rest_action();
		}
		void Put_Green()//放绿环——第一层
		{
			WritePosEx(1, Green.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			WritePosEx(3, Green.Num3, 3250, 60);
			WritePosEx(2, Green.Num2, 3250, 60);//向前伸
			HAL_Delay(800);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Blue()//放蓝环——第一层
		{
			WritePosEx(1, Blue.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(1000);
			WritePosEx(2, Blue.Num2, 3250, 60);//向前伸
			WritePosEx(3, Blue.Num3, 3250, 60);
			HAL_Delay(800);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Red_Stacking()//放红环——码垛
		{
			WritePosEx(1, Red_Stacking.Num1, 2500,60);//转到红环方向
			HAL_Delay(1200);
			WritePosEx(2, Red_Stacking.Num2, 3250, 60);//向前伸
			WritePosEx(3, Red_Stacking.Num3, 3250, 60);
			HAL_Delay(500);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);//300
			Rest_action();
		}
		void Put_Green_Stacking()//放绿环——码垛
		{
			WritePosEx(1, Green_Stacking.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			WritePosEx(3, Green_Stacking.Num3, 4250, 60);
			WritePosEx(2, Green_Stacking.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Blue_Stacking()//放蓝环——码垛
		{
			WritePosEx(1, Blue_Stacking.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(800);
			WritePosEx(2, Blue_Stacking.Num2, 3250, 60);//向前伸
			WritePosEx(3, Blue_Stacking.Num3, 3250, 60);
			HAL_Delay(500);
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Get_Red()//抓红环物料
		{
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			WritePosEx(1, Red.Num1, 2500,60);//转到红环方向
			HAL_Delay(1200);
			WritePosEx(2, Red.Num2, 3250, 60);//向前伸
			WritePosEx(3, Red.Num3, 3250, 60);
			HAL_Delay(500);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}
		void Get_Green()//抓绿环物料
		{
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			WritePosEx(1, Green.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			WritePosEx(3, Green.Num3, 3250, 60);
			WritePosEx(2, Green.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}
		void Get_Blue()//抓蓝环物料
		{
			WritePosEx(4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			WritePosEx(1, Blue.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(800);
			WritePosEx(2, Blue.Num2, 3250, 60);//向前伸
			WritePosEx(3, Blue.Num3, 3250, 60);			
			HAL_Delay(500);
			WritePosEx(4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}