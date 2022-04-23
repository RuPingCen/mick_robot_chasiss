#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stdlib.h"
#include "math.h"
#include "IO_IIC.h"
#include "MPU6050.h" 
#include "Kalman_Filter.h"
#include "Particle_Filter.h"
#include "Scope_API.h"

 
extern  float OutData[4];//Scope 示波器变量
extern double GYRO_Z_angle;//偏航角积分 需要一个全局变量

 

double x_N=0.23,x_R=0.3;
double result =0;//输出结果变量
double Particle_dt = 0.025;// 25 ms
double x_P[Particle_M],x_P_update[Particle_M],z_update[Particle_M];//粒子值 、 状态估计值 、 输出估计
 
double p_w[Particle_M],cumsum_p_w[Particle_M];//粒子权重 、 输出估计
  
double pi =3.1415926,init_particle_weirht = 1/(double)Particle_M;//初始权重
int i =0,j=0, sum_tem =0,rand_tem=0;//数组元素下标

int16_t Particle_int16datatemp[6];//滤波器16带符号数中间变量 用来读取6050传出的加速度计和陀螺仪数据
volatile double Particle_doubledatatemp[6];//滤波器浮点数中间变量 用来存储 三轴加速度计算出的角度  和 三轴陀螺仪 

//**************************************************************************************************// 
      
//可以使用6050数据输入 Particle 滤波器
 
void Particle_MPU6050data_prepare(void)
{

        MPU6050_Data_Process(Particle_int16datatemp); //读取6050数据   

				Particle_doubledatatemp[0]=(double)Particle_int16datatemp[0]/16384.0f; //加速度变成G   2G 对应16位有符号数最大值  32768  
				Particle_doubledatatemp[1]=(double)Particle_int16datatemp[1]/16384.0f;
				Particle_doubledatatemp[2]=(double)Particle_int16datatemp[2]/16384.0f; 

        if(Particle_doubledatatemp[0] >= 1) Particle_doubledatatemp[0]=0.99;//限制加速度计输出的幅度 避免过冲！！
        else if(Particle_doubledatatemp[0] <= -1) Particle_doubledatatemp[0]=-0.99;
        else ;
        
        if(Particle_doubledatatemp[1] >= 1) Particle_doubledatatemp[1]=0.99;
        else if(Particle_doubledatatemp[1] <= -1) Particle_doubledatatemp[1]=-0.99;
        else ;
        
        if(Particle_doubledatatemp[2] >= 1) Particle_doubledatatemp[2]=0.99;
        else if(Particle_doubledatatemp[2] <= -1) Particle_doubledatatemp[2]=-0.99;
        else ; 
        
       
				Particle_doubledatatemp[0] =asin(Particle_doubledatatemp[0])*57.3;//利用反三角函数计算角度  
				Particle_doubledatatemp[1] =asin(Particle_doubledatatemp[1])*57.3;
			//	Particle_doubledatatemp[2] =asin(Particle_doubledatatemp[2])*57.3;
   
 
       Particle_doubledatatemp[3]=(double)(Particle_int16datatemp[3])*0.061035; //6050陀螺仪的数据 转换为弧度        
       Particle_doubledatatemp[4]=(double)(Particle_int16datatemp[4])*0.061035; // Gyro_Y			 
       GYRO_Z_angle+=(float)(Particle_int16datatemp[5])*0.061035*Particle_dt; //Gyro_Z
       
       if(GYRO_Z_angle >= 360) GYRO_Z_angle=GYRO_Z_angle-360;
       else if(GYRO_Z_angle <= -360) GYRO_Z_angle=GYRO_Z_angle+360;
       else ;




//UART_send_floatdat(USART1,Particle_doubledatatemp[0]);UART_send_char(USART1,'\t');
//UART_send_floatdat(USART1,Particle_doubledatatemp[1]);UART_send_char(USART1,'\t');
//UART_send_floatdat(USART1,Particle_doubledatatemp[3]);UART_send_char(USART1,'\t');
//UART_send_floatdat(USART1,Particle_doubledatatemp[4]);UART_send_char(USART1,'\n');
 


}

/***********************************************************************************
 *
 *粒子算法实现 粒子数M=50 计算时间13ms
 *
 *maker: cen ru ping
 *
 *Data: 2016-7-10
 *
 *备注：积分存在静差--积分出来的角度与真实的角度对应不上
***********************************************************************************/
void Particle_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y)// 粒子滤波算法实现函数
{

	if(i==0)//程序第一次运行进行初始化
	{
		for(i=0;i<Particle_M;i++)//初始化粒子权重
		{
				x_P[i]=init_particle_weirht;

		}
	}
	
	sum_tem = 0;
	for(i=0;i<Particle_M;i++)
	{	 
		x_P_update[i] = x_P[i] - GYRO_Y*Particle_dt +x_N*(rand()/(double)Random_Data_MAX-0.5);// 从先验p(x(k)|x(k-1))中采样 利用0均值高斯噪声做扰动
		z_update[i] = x_P_update[i] +x_R*(rand()/(double)Random_Data_MAX-0.5);
		p_w[i] = (double) 1/(double)sqrt(2*pi*x_R) * exp(-(Acc_angl_x  - z_update[i])*(Acc_angl_x  - z_update[i])/(double)(2*x_R)); //Gauss distribution
		
		sum_tem+=p_w[i];
	}
	
	for(i=0;i<Particle_M;i++)// 
	{
		p_w[i]=p_w[i]/(double)sum_tem;//做归一化
		cumsum_p_w[i]= 0;
		for(j=0;j<=i;j++)
			cumsum_p_w[i]+=p_w[j];//对粒子进行重采样 matlab cumsum函数的实现

	}

// 查找第一个不为0 的元素下标
	for(i=0;i<Particle_M;i++)//初始化粒子权重
	{
		rand_tem =rand()/(double)Random_Data_MAX;
		for(j=0;j<Particle_M;j++)
			if(rand_tem <=cumsum_p_w[j] )
			{
				x_P[i]=x_P_update[j];
				break;;
			}
	}
	// 求取均值作为最优输出
 	result=0;
  for(i=0;i<Particle_M;i++)//初始化粒子权重
	{
		result+=x_P[i];
	}
	result =result/(double)Particle_M;

 
}
 

void Particle_Process(double out_angle[3],int16_t GYRO[3])//在定时器里面调用这个函数就可以了
{ 
 
	Particle_MPU6050data_prepare();//利用6050数据传入 滤波器进行滤波
	GPIO_SetBits(GPIOF,GPIO_Pin_7);//置位一个引脚  使引脚输出‘1’

	Particle_Filter(Particle_doubledatatemp[3],Particle_doubledatatemp[4],Particle_doubledatatemp[0],Particle_doubledatatemp[1]);
	Kalman_Filter(Particle_doubledatatemp[3],-Particle_doubledatatemp[4],Particle_doubledatatemp[0],Particle_doubledatatemp[1]);//临时测试使用
 
	OutData[0]=Particle_doubledatatemp[0]; //将计算的角度输出  X
	OutData[1]=result; // 
 	

  OutPut_Data();
	GPIO_ResetBits(GPIOF,GPIO_Pin_7);//复位一个引脚  使引脚输出‘0’
}
