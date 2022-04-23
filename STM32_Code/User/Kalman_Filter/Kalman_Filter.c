#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "math.h"
#include "IO_IIC.h"
#include "MPU6050.h" 
#include "Kalman_Filter.h"
#include "Scope_API.h"

//#include "STM32f40x_UART.h"
//#include "stm32f4xx_usart.h"

extern  float OutData[4];
 
int16_t Kalman_int16datatemp[6];//滤波器16带符号数中间变量 用来读取6050传出的加速度计和陀螺仪数据
volatile double Kalman_doubledatatemp[6];//滤波器浮点数中间变量 用来存储 三轴加速度计算出的角度  和 三轴陀螺仪 
volatile double GYRO_Z_angle=0;//偏航角积分 需要一个全局变量

//*************************************卡尔曼滤波参数************************************************//
//**************************************************************************************************//
  volatile  double  Acc_angle,Gry_vivi;
  volatile  double  Angle[2]={0,0},Gyro_x[2]={0,0};         //小车滤波后倾斜角度/角速度	
  volatile  double  Q_angle=0.001;  
  volatile  double  Q_gyro=0.003;
  volatile  double  R_angle=0.5;
  volatile  double  dt=0.0187;	                  //dt为kalman滤波器采样时间;
  volatile  char    C[2]={1,1};
  volatile  double  Q_bias[2]={0,0}, Angle_err[2]={0,0};
  volatile  double  PCt[2][2]={{0,0},{0,0}},E=0;
  volatile  double  K[2][2]={{0,0},{0,0}},t[2][2]={{0,0},{0,0}};
  volatile  double  Pdot[2][4] ={{0,0,0,0},{0,0,0,0}};
  volatile  double  PP[4][2] = { { 1, 0 },{ 0, 1 },{ 1, 0 },{ 0, 1 } };      
        
//**************************************************************************************************// 
      
//可以使用6050数据输入 Kalman 滤波器
 
void Kalman_MPU6050data_prepare(void)
{

        MPU6050_Data_Process(Kalman_int16datatemp); //读取6050数据   

				Kalman_doubledatatemp[0]=(double)Kalman_int16datatemp[0]/16384.0f; //加速度变成G   2G 对应16位有符号数最大值  32768  
				Kalman_doubledatatemp[1]=(double)Kalman_int16datatemp[1]/16384.0f;
				Kalman_doubledatatemp[2]=(double)Kalman_int16datatemp[2]/16384.0f; 

        if(Kalman_doubledatatemp[0] >= 1) Kalman_doubledatatemp[0]=0.99;//限制加速度计输出的幅度 避免过冲！！
        else if(Kalman_doubledatatemp[0] <= -1) Kalman_doubledatatemp[0]=-0.99;
        else ;
        
        if(Kalman_doubledatatemp[1] >= 1) Kalman_doubledatatemp[1]=0.99;
        else if(Kalman_doubledatatemp[1] <= -1) Kalman_doubledatatemp[1]=-0.99;
        else ;
        
        if(Kalman_doubledatatemp[2] >= 1) Kalman_doubledatatemp[2]=0.99;
        else if(Kalman_doubledatatemp[2] <= -1) Kalman_doubledatatemp[2]=-0.99;
        else ; 
        
       
				Kalman_doubledatatemp[0] =asin(Kalman_doubledatatemp[0])*57.3;//利用反三角函数计算角度  
				Kalman_doubledatatemp[1] =asin(Kalman_doubledatatemp[1])*57.3;
			//	Kalman_doubledatatemp[2] =asin(Kalman_doubledatatemp[2])*57.3;
   
 
       Kalman_doubledatatemp[3]=(double)(Kalman_int16datatemp[3])*0.001065; //6050陀螺仪的数据 转换为弧度        
       Kalman_doubledatatemp[4]=(double)(Kalman_int16datatemp[4])*0.001065; // Gyro_Y			 
       GYRO_Z_angle+=(float)(Kalman_int16datatemp[5])*0.061035*dt; //Gyro_Z
       
       if(GYRO_Z_angle >= 360) GYRO_Z_angle=GYRO_Z_angle-360;
       else if(GYRO_Z_angle <= -360) GYRO_Z_angle=GYRO_Z_angle+360;
       else ;
}


 

void Kalman_Process(double out_angle[3],int16_t GYRO[3])//在定时器里面调用这个函数就可以了
{ 
 
		Kalman_MPU6050data_prepare();//利用6050数据传入Kalman 滤波器进行滤波
		//GPIO_SetBits(GPIOF,GPIO_Pin_7);//置位一个引脚  使引脚输出‘1’

		Kalman_Filter(Kalman_doubledatatemp[3], Kalman_doubledatatemp[4],Kalman_doubledatatemp[0],Kalman_doubledatatemp[1]);
		// Kalman_Filter_Verify(Kalman_doubledatatemp[4],Kalman_doubledatatemp[0]);

		//GPIO_ResetBits(GPIOF,GPIO_Pin_7);//复位一个引脚  使引脚输出‘0’

		out_angle[0]=Angle[0]; //将计算的角度输出  X
		out_angle[1]=Angle[1]; //                  Y
		out_angle[2]=GYRO_Z_angle; //偏航角

		GYRO[0]=Gyro_x[1];//X方向陀螺仪
		GYRO[1]=Gyro_x[0];//Y方向陀螺仪
		GYRO[2]=Kalman_int16datatemp[5];

	 	OutData[0]=out_angle[0];
	 	OutData[1]=out_angle[1];
		OutData[2]=Kalman_doubledatatemp[0] ;//输出滤波前的角度
		OutData[3]=Kalman_doubledatatemp[1] ;
	//  OutData[2]=Gyro_x[1] ; 
	//	OutData[3]=Gyro_x[0] ;
  	OutPut_Data(); 

	//	UART_send_floatdat(USART1,out_angle[0]);UART_send_char(USART1,'\t');UART_send_floatdat(USART1,Gyro_x[1]);UART_send_char(USART1,'\t');
	//	UART_send_floatdat(USART1,out_angle[1]);UART_send_char(USART1,'\t');UART_send_floatdat(USART1,Gyro_x[0]);UART_send_char(USART1,'\n');
  //  UART_send_floatdat(USART1,out_angle[2]);UART_send_char(USART1,'\n');
	
}




  


/*************************************************************
  函数说明:卡尔曼滤波函数                                    *
                                                             *       
  参数： Gyro代表陀螺仪    Accel 代表由加速度计算出来的角度  *
                                                             *
  传出参数：out_angle[0]、out_angle[1]--角度    Gyro_x 角速度* 
                                                             * 
**************************************************************/
void Kalman_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y)//Kalman滤波，100MHz的处理时间约****ms；	
{    
  Angle[0]+=(GYRO_Y - Q_bias[0])*dt;           //先验估计
	
	Pdot[0][0]=Q_angle-PP[0][1]-PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[0][1]=- PP[1][1];
	Pdot[0][2]=- PP[1][1];
	Pdot[0][3]=Q_gyro;

	PP[0][0] += Pdot[0][0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[0][1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[0][2] * dt;
	PP[1][1] += Pdot[0][3] * dt;
		
	Angle_err[0]=Acc_angl_x - Angle[0];	//zk-先验估计
	
	PCt[0][0] = C[0] * PP[0][0];
	PCt[0][1] = C[0] * PP[1][0];	
	E = R_angle +C[0] * PCt[0][0];	
	K[0][0] = PCt[0][0] / E;
	K[0][1] = PCt[0][1] / E;
	
	t[0][0] = PCt[0][0];
	t[0][1] = C[0] * PP[0][1];

	PP[0][0] -= K[0][0] * t[0][0];		 //后验估计误差协方差
	PP[0][1] -= K[0][0] * t[0][1];
	PP[1][0] -= K[0][1] * t[0][0];
	PP[1][1] -= K[0][1] * t[0][1];
	
 
	
		
	Angle[0]+= K[0][0] * Angle_err[0];	 //后验估计
	Q_bias[0]+= K[0][1] * Angle_err[0];	 //后验估计
	Gyro_x[0]= GYRO_Y - Q_bias[0];	 //输出值(后验估计)的微分=角速度
	
	
	
	
	Angle[1]+=(GYRO_X - Q_bias[1])*dt;           //先验估计
	
	Pdot[1][0]=Q_angle-PP[2][1]-PP[3][0]; // Pk-先验估计误差协方差的微分

	Pdot[1][1]=- PP[3][1];
	Pdot[1][2]=- PP[3][1];
	Pdot[1][3]=Q_gyro;

	PP[2][0] += Pdot[1][0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[2][1] += Pdot[1][1] * dt;   // =先验估计误差协方差
	PP[3][0] += Pdot[1][2] * dt;
	PP[3][1] += Pdot[1][3] * dt;
		
	Angle_err[1]=Acc_angl_y - Angle[1];	//zk-先验估计
	
	PCt[1][0] = C[1] * PP[2][0];
	PCt[1][1] = C[1] * PP[3][0];	
	E = R_angle +C[1] * PCt[1][0];	
	K[1][0] = PCt[1][0] / E;
	K[1][1] = PCt[1][1] / E;
	
	t[1][0] = PCt[1][0];
	t[1][1] = C[1] * PP[2][1];

	PP[2][0] -= K[1][0] * t[1][0];		 //后验估计误差协方差
	PP[2][1] -= K[1][0] * t[1][1];
	PP[3][0] -= K[1][1] * t[1][0];
	PP[3][1] -= K[1][1] * t[1][1];
		
	Angle[1]+= K[1][0] * Angle_err[1];	 //后验估计 Y轴偏离角
	Q_bias[1]+= K[1][1] * Angle_err[1];	 //后验估计
	Gyro_x[1]= GYRO_X - Q_bias[1];	 //输出值(后验估计)的微分=角速度	

       
} 

 
 //该函数的目的主要是核实卡尔曼P 矩阵的计算方式
//实验证明：Pk+1 = APk + PkA'+Q 这种方式好像是不对的
//
// crp
// 2016 01 09  
void Kalman_Filter_Verify(float GYRO_Y,float Acc_angl_x)//Kalman滤波，100MHz的处理时间约****ms；	
{     
  Angle[0]+=(GYRO_Y - Q_bias[0])*dt;           //先验估计
	
//	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt + PP[1][1]* dt* dt + Q_angle; // Pk+1 = APkA'+Q
	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt  + Q_angle; // （可以忽略掉 PP[1][1]* dt* dt 项 因为其太小了）
	PP[0][1] = PP[0][1] - PP[1][1] * dt;//该方法动态效果还不错  推荐使用 
	PP[1][0] = PP[1][0] - PP[1][1] * dt;// Q 是一个对角阵
	PP[1][1] = PP[1][1] + Q_gyro;
	
//	PP[0][0] = PP[0][0]*2 - PP[0][1]* dt - PP[1][0]* dt + Q_angle; // Pk+1 = APk + PkA'+Q
//	PP[0][1] = PP[0][1]*2 - PP[1][1] * dt;//实际检测效果不好   
//	PP[1][0] = PP[1][0]*2 - PP[1][1] * dt;// Q 是一个对角阵
//	PP[1][1] = PP[1][1]*2 + Q_gyro;	
	
//	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt + Q_angle* dt; // 使用时候的计算方式 效果不错
//	PP[0][1] = PP[0][1] - PP[1][1] * dt;   
//	PP[1][0] = PP[1][0] - PP[1][1] * dt;// Q 是一个对角阵
//	PP[1][1] = PP[1][1] + Q_gyro* dt;		

  
	Angle_err[0]=Acc_angl_x - Angle[0];	//观测值

	K[0][0] = C[0] * PP[0][0]/(R_angle + PP[0][0]);//计算卡尔曼增益
	K[0][1] = C[0] * PP[0][1]/(R_angle + PP[0][0]);
	
	Angle[0]+= K[0][0] * Angle_err[0];	 //后验估计
	Q_bias[0]+= K[0][1] * Angle_err[0];	 //后验估计
	Gyro_x[0]= GYRO_Y - Q_bias[0];	 //输出值(后验估计)的微分=角速度	
	

	PP[1][0] = PP[1][0] - K[0][1] * PP[0][0];//后验估计误差协方差
	PP[1][1] = PP[1][1] - K[0][1] * PP[0][1];
	PP[0][0] = PP[0][0] * (1-K[0][0]);		 
	PP[0][1] = PP[0][1] * (1-K[0][0]);
 
	
} 
