#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "PID.h"
#include "BlueSerial.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*速度环角度环串级+转向环pid*/
int16_t AX, AY, AZ, GX, GY, GZ;//六轴数据量
uint8_t TimerErrorFlag;
uint16_t TimerCount;

float AngleAcc;//加速度测角度
float AngleGyro;//陀螺仪测角度
float Angle;//互补滤波测角度

uint8_t KeyNum, RunFlag = 0;//按键，控制标志位

int16_t LeftPWM, RightPWM;//左右轮PWM
int16_t AvePWM, DifPWM;//平均pwm，差分pwm

float LeftSpeed, RightSpeed;//左右轮速度
float AveSpeed, DifSpeed;//平均速度与差分速度

PID_t AnglePID = {        //角度结构体
		.Kp = 5.0,
		.Ki = 0.28,
		.Kd = 3.0,
	
		.OutMax = 100,
		.OutMin = -100,
	
};

PID_t SpeedPID = {        //速度结构体
		.Kp = 1.30,
		.Ki = 0.08,
		.Kd = 0,
	
		.OutMax = 15,  //角度环角度范围
		.OutMin = -15,
	
};

PID_t TurnPID = {        //转向结构体
		.Kp = 0,
		.Ki = 0,
		.Kd = 0,
	
		.OutMax = 50,
		.OutMin = -50,
	
};


int main(void)
{
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	
	
	Timer_Init();
	
	while (1)
	{
		if(RunFlag){LED_ON();}//led指示启停状态
		else{LED_OFF();}
		
		KeyNum = Key_GetNum();//按键一控制程序启停
		if(KeyNum == 1){
			if(RunFlag == 0){
				//清零积分等参数
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag = 1;
				
			}
			else{
				RunFlag = 0;
			}
		}
		
		
		OLED_Clear();
		
		OLED_Printf(0, 0, OLED_6X8, "  Angle", AX);
		OLED_Printf(0, 8, OLED_6X8, "p:%05.2f", AnglePID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%05.2f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%05.1f", Angle);
		OLED_Printf(0, 48, OLED_6X8, "S:%05.1f", AnglePID.Out);
		OLED_Printf(0, 56, OLED_6X8, "GX:%05.1f", GY);
		
		OLED_Printf(50, 0, OLED_6X8, "  Speed", AX);
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(50, 32, OLED_6X8, "%05.2f", SpeedPID.Target);
		OLED_Printf(50, 40, OLED_6X8, "%05.1f", AveSpeed);
		OLED_Printf(50, 48, OLED_6X8, "%05.1f", SpeedPID.Out);
		
		OLED_Printf(88, 0, OLED_6X8, "  Turn", AX);
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(88, 32, OLED_6X8, "%05.2f", TurnPID.Target);
		OLED_Printf(88, 40, OLED_6X8, "%05.1f", DifSpeed);
		OLED_Printf(88, 48, OLED_6X8, "%05.1f", TurnPID.Out);
		
		OLED_Update();
		
		//蓝牙模块调参pid
		
		if (BlueSerial_RxFlag == 1)
		{
			//strtok strcmp atoi/atof
			
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if(strcmp(Name, "AngleKp") == 0){
					AnglePID.Kp = atof(Value);
				}
				
				else if(strcmp(Name, "AngleKi") == 0){
					AnglePID.Ki = atof(Value);
				}
				
				else if(strcmp(Name, "AngleKd") == 0){
					AnglePID.Kd = atof(Value);
				}
				
				
				
				else if(strcmp(Name, "SpeedKp") == 0){
					SpeedPID.Kp = atof(Value);
				}
				
				else if(strcmp(Name, "SpeedKi") == 0){
					SpeedPID.Ki = atof(Value);
				}
				
				else if(strcmp(Name, "SpeedKd") == 0){
					SpeedPID.Kd = atof(Value);
				}
				
				
				
				else if(strcmp(Name, "TurnKp") == 0){
					TurnPID.Kp = atof(Value);
				}
				
				else if(strcmp(Name, "TurnKi") == 0){
					TurnPID.Ki = atof(Value);
				}
				
				else if(strcmp(Name, "TurnKd") == 0){
					TurnPID.Kd = atof(Value);
				}
			}
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target = LV / 25.0;
				TurnPID.Target = RH / 25.0;
			}
			
			BlueSerial_RxFlag = 0;
		}

		
		BlueSerial_Printf("[plot,%f,%f]", TurnPID.Target, DifSpeed);
	}
}

void TIM1_UP_IRQHandler(void) //1ms进一次中断
{
	static uint16_t Count0;//角度环调控周期10ms
	static uint16_t Count1;//速度环读取编码器周期,转向环周期
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		Count0 ++;
		if (Count0 >= 10){
			Count0 = 0;
			
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);//计算俯仰角
		
			GX -= 110;//消除稳态误差
			
			//加速度,校准
			AngleAcc = -atan2(AY,AZ)/3.14159 * 180;
			AngleAcc -= 2.7;
			//陀螺仪
			AngleGyro = Angle - GX /32768.0 * 2000 * 0.01;
			//互补滤波
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1-Alpha) * AngleGyro;
			
			if(Angle > 40 || Angle < -40){RunFlag = 0;}
			
			//角度环pid
			if(RunFlag){
				//pid调控
				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;
				
				//变量公式
				LeftPWM = AvePWM + DifPWM / 2;
				RightPWM = AvePWM - DifPWM / 2;
				
				//pwm限幅
				if(LeftPWM > 100){LeftPWM = 100;} else if(LeftPWM < -100){LeftPWM = -100;}
				if(RightPWM > 100){RightPWM = 100;} else if(RightPWM < -100){RightPWM = -100;}
				
				//电机设置pwm
				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else{
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
		
		Count1++;
		if(Count1 >= 50){
			Count1 = 0;
			
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;

			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;
			
			if (RunFlag)
			{
					SpeedPID.Actual = AveSpeed;
					PID_Update(&SpeedPID);
					AnglePID.Target = SpeedPID.Out;
				
					TurnPID.Actual = DifSpeed;
					PID_Update(&TurnPID);
					DifPWM = TurnPID.Out;
			}


			
		}
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}



/*加速度测量俯仰角，缺陷严重不采用*/
//int16_t AX, AY, AZ, GX, GY, GZ;
//uint8_t TimerErrorFlag;
//uint16_t TimerCount;

//float AngleAcc;


//int main(void)
//{
//	OLED_Init();
//	MPU6050_Init();
//	BlueSerial_Init();
//	
//	
//	Timer_Init();
//	
//	while (1)
//	{
//		
//		OLED_Printf(0, 0, OLED_8X16, "%+06d", AX);
//		OLED_Printf(0, 16, OLED_8X16, "%+06d", AY);
//		OLED_Printf(0, 32, OLED_8X16, "%+06d", AZ);
//		OLED_Printf(64, 0, OLED_8X16, "%+06d", GX);
//		OLED_Printf(64, 16, OLED_8X16, "%+06d", GY);
//		OLED_Printf(64, 32, OLED_8X16, "%+06d", GZ);
//		OLED_Printf(0, 48, OLED_8X16, "Flag:%1d", TimerErrorFlag);
//		OLED_Printf(64, 48, OLED_8X16, "C:%05d", TimerCount);
//		OLED_Update();
//		
//		BlueSerial_Printf("[plot,%f]", AngleAcc);
//	}
//}

//void TIM1_UP_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
//	{
//		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//		
//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//		
//		AngleAcc = atan2(AY,AZ)/3.14159*180;
//		
//		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
//		{
//			TimerErrorFlag = 1;
//			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//		}
//		TimerCount = TIM_GetCounter(TIM1);
//	}
//}




