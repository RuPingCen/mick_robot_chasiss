#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stdlib.h"
#include "math.h"
#include "IO_IIC.h"
#include "MPU6050.h" 
#include "Kalman_Filter.h"
#include "Particle_Filter.h"
#include "Scope_API.h"

 
extern  float OutData[4];//Scope ʾ��������
extern double GYRO_Z_angle;//ƫ���ǻ��� ��Ҫһ��ȫ�ֱ���

 

double x_N=0.23,x_R=0.3;
double result =0;//����������
double Particle_dt = 0.025;// 25 ms
double x_P[Particle_M],x_P_update[Particle_M],z_update[Particle_M];//����ֵ �� ״̬����ֵ �� �������
 
double p_w[Particle_M],cumsum_p_w[Particle_M];//����Ȩ�� �� �������
  
double pi =3.1415926,init_particle_weirht = 1/(double)Particle_M;//��ʼȨ��
int i =0,j=0, sum_tem =0,rand_tem=0;//����Ԫ���±�

int16_t Particle_int16datatemp[6];//�˲���16���������м���� ������ȡ6050�����ļ��ٶȼƺ�����������
volatile double Particle_doubledatatemp[6];//�˲����������м���� �����洢 ������ٶȼ�����ĽǶ�  �� ���������� 

//**************************************************************************************************// 
      
//����ʹ��6050�������� Particle �˲���
 
void Particle_MPU6050data_prepare(void)
{

        MPU6050_Data_Process(Particle_int16datatemp); //��ȡ6050����   

				Particle_doubledatatemp[0]=(double)Particle_int16datatemp[0]/16384.0f; //���ٶȱ��G   2G ��Ӧ16λ�з��������ֵ  32768  
				Particle_doubledatatemp[1]=(double)Particle_int16datatemp[1]/16384.0f;
				Particle_doubledatatemp[2]=(double)Particle_int16datatemp[2]/16384.0f; 

        if(Particle_doubledatatemp[0] >= 1) Particle_doubledatatemp[0]=0.99;//���Ƽ��ٶȼ�����ķ��� ������壡��
        else if(Particle_doubledatatemp[0] <= -1) Particle_doubledatatemp[0]=-0.99;
        else ;
        
        if(Particle_doubledatatemp[1] >= 1) Particle_doubledatatemp[1]=0.99;
        else if(Particle_doubledatatemp[1] <= -1) Particle_doubledatatemp[1]=-0.99;
        else ;
        
        if(Particle_doubledatatemp[2] >= 1) Particle_doubledatatemp[2]=0.99;
        else if(Particle_doubledatatemp[2] <= -1) Particle_doubledatatemp[2]=-0.99;
        else ; 
        
       
				Particle_doubledatatemp[0] =asin(Particle_doubledatatemp[0])*57.3;//���÷����Ǻ�������Ƕ�  
				Particle_doubledatatemp[1] =asin(Particle_doubledatatemp[1])*57.3;
			//	Particle_doubledatatemp[2] =asin(Particle_doubledatatemp[2])*57.3;
   
 
       Particle_doubledatatemp[3]=(double)(Particle_int16datatemp[3])*0.061035; //6050�����ǵ����� ת��Ϊ����        
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
 *�����㷨ʵ�� ������M=50 ����ʱ��13ms
 *
 *maker: cen ru ping
 *
 *Data: 2016-7-10
 *
 *��ע�����ִ��ھ���--���ֳ����ĽǶ�����ʵ�ĽǶȶ�Ӧ����
***********************************************************************************/
void Particle_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y)// �����˲��㷨ʵ�ֺ���
{

	if(i==0)//�����һ�����н��г�ʼ��
	{
		for(i=0;i<Particle_M;i++)//��ʼ������Ȩ��
		{
				x_P[i]=init_particle_weirht;

		}
	}
	
	sum_tem = 0;
	for(i=0;i<Particle_M;i++)
	{	 
		x_P_update[i] = x_P[i] - GYRO_Y*Particle_dt +x_N*(rand()/(double)Random_Data_MAX-0.5);// ������p(x(k)|x(k-1))�в��� ����0��ֵ��˹�������Ŷ�
		z_update[i] = x_P_update[i] +x_R*(rand()/(double)Random_Data_MAX-0.5);
		p_w[i] = (double) 1/(double)sqrt(2*pi*x_R) * exp(-(Acc_angl_x  - z_update[i])*(Acc_angl_x  - z_update[i])/(double)(2*x_R)); //Gauss distribution
		
		sum_tem+=p_w[i];
	}
	
	for(i=0;i<Particle_M;i++)// 
	{
		p_w[i]=p_w[i]/(double)sum_tem;//����һ��
		cumsum_p_w[i]= 0;
		for(j=0;j<=i;j++)
			cumsum_p_w[i]+=p_w[j];//�����ӽ����ز��� matlab cumsum������ʵ��

	}

// ���ҵ�һ����Ϊ0 ��Ԫ���±�
	for(i=0;i<Particle_M;i++)//��ʼ������Ȩ��
	{
		rand_tem =rand()/(double)Random_Data_MAX;
		for(j=0;j<Particle_M;j++)
			if(rand_tem <=cumsum_p_w[j] )
			{
				x_P[i]=x_P_update[j];
				break;;
			}
	}
	// ��ȡ��ֵ��Ϊ�������
 	result=0;
  for(i=0;i<Particle_M;i++)//��ʼ������Ȩ��
	{
		result+=x_P[i];
	}
	result =result/(double)Particle_M;

 
}
 

void Particle_Process(double out_angle[3],int16_t GYRO[3])//�ڶ�ʱ�����������������Ϳ�����
{ 
 
	Particle_MPU6050data_prepare();//����6050���ݴ��� �˲��������˲�
	GPIO_SetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������1��

	Particle_Filter(Particle_doubledatatemp[3],Particle_doubledatatemp[4],Particle_doubledatatemp[0],Particle_doubledatatemp[1]);
	Kalman_Filter(Particle_doubledatatemp[3],-Particle_doubledatatemp[4],Particle_doubledatatemp[0],Particle_doubledatatemp[1]);//��ʱ����ʹ��
 
	OutData[0]=Particle_doubledatatemp[0]; //������ĽǶ����  X
	OutData[1]=result; // 
 	

  OutPut_Data();
	GPIO_ResetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������0��
}
