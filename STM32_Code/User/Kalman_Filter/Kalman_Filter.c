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
 
int16_t Kalman_int16datatemp[6];//�˲���16���������м���� ������ȡ6050�����ļ��ٶȼƺ�����������
volatile double Kalman_doubledatatemp[6];//�˲����������м���� �����洢 ������ٶȼ�����ĽǶ�  �� ���������� 
volatile double GYRO_Z_angle=0;//ƫ���ǻ��� ��Ҫһ��ȫ�ֱ���

//*************************************�������˲�����************************************************//
//**************************************************************************************************//
  volatile  double  Acc_angle,Gry_vivi;
  volatile  double  Angle[2]={0,0},Gyro_x[2]={0,0};         //С���˲�����б�Ƕ�/���ٶ�	
  volatile  double  Q_angle=0.001;  
  volatile  double  Q_gyro=0.003;
  volatile  double  R_angle=0.5;
  volatile  double  dt=0.0187;	                  //dtΪkalman�˲�������ʱ��;
  volatile  char    C[2]={1,1};
  volatile  double  Q_bias[2]={0,0}, Angle_err[2]={0,0};
  volatile  double  PCt[2][2]={{0,0},{0,0}},E=0;
  volatile  double  K[2][2]={{0,0},{0,0}},t[2][2]={{0,0},{0,0}};
  volatile  double  Pdot[2][4] ={{0,0,0,0},{0,0,0,0}};
  volatile  double  PP[4][2] = { { 1, 0 },{ 0, 1 },{ 1, 0 },{ 0, 1 } };      
        
//**************************************************************************************************// 
      
//����ʹ��6050�������� Kalman �˲���
 
void Kalman_MPU6050data_prepare(void)
{

        MPU6050_Data_Process(Kalman_int16datatemp); //��ȡ6050����   

				Kalman_doubledatatemp[0]=(double)Kalman_int16datatemp[0]/16384.0f; //���ٶȱ��G   2G ��Ӧ16λ�з��������ֵ  32768  
				Kalman_doubledatatemp[1]=(double)Kalman_int16datatemp[1]/16384.0f;
				Kalman_doubledatatemp[2]=(double)Kalman_int16datatemp[2]/16384.0f; 

        if(Kalman_doubledatatemp[0] >= 1) Kalman_doubledatatemp[0]=0.99;//���Ƽ��ٶȼ�����ķ��� ������壡��
        else if(Kalman_doubledatatemp[0] <= -1) Kalman_doubledatatemp[0]=-0.99;
        else ;
        
        if(Kalman_doubledatatemp[1] >= 1) Kalman_doubledatatemp[1]=0.99;
        else if(Kalman_doubledatatemp[1] <= -1) Kalman_doubledatatemp[1]=-0.99;
        else ;
        
        if(Kalman_doubledatatemp[2] >= 1) Kalman_doubledatatemp[2]=0.99;
        else if(Kalman_doubledatatemp[2] <= -1) Kalman_doubledatatemp[2]=-0.99;
        else ; 
        
       
				Kalman_doubledatatemp[0] =asin(Kalman_doubledatatemp[0])*57.3;//���÷����Ǻ�������Ƕ�  
				Kalman_doubledatatemp[1] =asin(Kalman_doubledatatemp[1])*57.3;
			//	Kalman_doubledatatemp[2] =asin(Kalman_doubledatatemp[2])*57.3;
   
 
       Kalman_doubledatatemp[3]=(double)(Kalman_int16datatemp[3])*0.001065; //6050�����ǵ����� ת��Ϊ����        
       Kalman_doubledatatemp[4]=(double)(Kalman_int16datatemp[4])*0.001065; // Gyro_Y			 
       GYRO_Z_angle+=(float)(Kalman_int16datatemp[5])*0.061035*dt; //Gyro_Z
       
       if(GYRO_Z_angle >= 360) GYRO_Z_angle=GYRO_Z_angle-360;
       else if(GYRO_Z_angle <= -360) GYRO_Z_angle=GYRO_Z_angle+360;
       else ;
}


 

void Kalman_Process(double out_angle[3],int16_t GYRO[3])//�ڶ�ʱ�����������������Ϳ�����
{ 
 
		Kalman_MPU6050data_prepare();//����6050���ݴ���Kalman �˲��������˲�
		//GPIO_SetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������1��

		Kalman_Filter(Kalman_doubledatatemp[3], Kalman_doubledatatemp[4],Kalman_doubledatatemp[0],Kalman_doubledatatemp[1]);
		// Kalman_Filter_Verify(Kalman_doubledatatemp[4],Kalman_doubledatatemp[0]);

		//GPIO_ResetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������0��

		out_angle[0]=Angle[0]; //������ĽǶ����  X
		out_angle[1]=Angle[1]; //                  Y
		out_angle[2]=GYRO_Z_angle; //ƫ����

		GYRO[0]=Gyro_x[1];//X����������
		GYRO[1]=Gyro_x[0];//Y����������
		GYRO[2]=Kalman_int16datatemp[5];

	 	OutData[0]=out_angle[0];
	 	OutData[1]=out_angle[1];
		OutData[2]=Kalman_doubledatatemp[0] ;//����˲�ǰ�ĽǶ�
		OutData[3]=Kalman_doubledatatemp[1] ;
	//  OutData[2]=Gyro_x[1] ; 
	//	OutData[3]=Gyro_x[0] ;
  	OutPut_Data(); 

	//	UART_send_floatdat(USART1,out_angle[0]);UART_send_char(USART1,'\t');UART_send_floatdat(USART1,Gyro_x[1]);UART_send_char(USART1,'\t');
	//	UART_send_floatdat(USART1,out_angle[1]);UART_send_char(USART1,'\t');UART_send_floatdat(USART1,Gyro_x[0]);UART_send_char(USART1,'\n');
  //  UART_send_floatdat(USART1,out_angle[2]);UART_send_char(USART1,'\n');
	
}




  


/*************************************************************
  ����˵��:�������˲�����                                    *
                                                             *       
  ������ Gyro����������    Accel �����ɼ��ٶȼ�������ĽǶ�  *
                                                             *
  ����������out_angle[0]��out_angle[1]--�Ƕ�    Gyro_x ���ٶ�* 
                                                             * 
**************************************************************/
void Kalman_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y)//Kalman�˲���100MHz�Ĵ���ʱ��Լ****ms��	
{    
  Angle[0]+=(GYRO_Y - Q_bias[0])*dt;           //�������
	
	Pdot[0][0]=Q_angle-PP[0][1]-PP[1][0]; // Pk-����������Э�����΢��

	Pdot[0][1]=- PP[1][1];
	Pdot[0][2]=- PP[1][1];
	Pdot[0][3]=Q_gyro;

	PP[0][0] += Pdot[0][0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[0][1] * dt;   // =����������Э����
	PP[1][0] += Pdot[0][2] * dt;
	PP[1][1] += Pdot[0][3] * dt;
		
	Angle_err[0]=Acc_angl_x - Angle[0];	//zk-�������
	
	PCt[0][0] = C[0] * PP[0][0];
	PCt[0][1] = C[0] * PP[1][0];	
	E = R_angle +C[0] * PCt[0][0];	
	K[0][0] = PCt[0][0] / E;
	K[0][1] = PCt[0][1] / E;
	
	t[0][0] = PCt[0][0];
	t[0][1] = C[0] * PP[0][1];

	PP[0][0] -= K[0][0] * t[0][0];		 //����������Э����
	PP[0][1] -= K[0][0] * t[0][1];
	PP[1][0] -= K[0][1] * t[0][0];
	PP[1][1] -= K[0][1] * t[0][1];
	
 
	
		
	Angle[0]+= K[0][0] * Angle_err[0];	 //�������
	Q_bias[0]+= K[0][1] * Angle_err[0];	 //�������
	Gyro_x[0]= GYRO_Y - Q_bias[0];	 //���ֵ(�������)��΢��=���ٶ�
	
	
	
	
	Angle[1]+=(GYRO_X - Q_bias[1])*dt;           //�������
	
	Pdot[1][0]=Q_angle-PP[2][1]-PP[3][0]; // Pk-����������Э�����΢��

	Pdot[1][1]=- PP[3][1];
	Pdot[1][2]=- PP[3][1];
	Pdot[1][3]=Q_gyro;

	PP[2][0] += Pdot[1][0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[2][1] += Pdot[1][1] * dt;   // =����������Э����
	PP[3][0] += Pdot[1][2] * dt;
	PP[3][1] += Pdot[1][3] * dt;
		
	Angle_err[1]=Acc_angl_y - Angle[1];	//zk-�������
	
	PCt[1][0] = C[1] * PP[2][0];
	PCt[1][1] = C[1] * PP[3][0];	
	E = R_angle +C[1] * PCt[1][0];	
	K[1][0] = PCt[1][0] / E;
	K[1][1] = PCt[1][1] / E;
	
	t[1][0] = PCt[1][0];
	t[1][1] = C[1] * PP[2][1];

	PP[2][0] -= K[1][0] * t[1][0];		 //����������Э����
	PP[2][1] -= K[1][0] * t[1][1];
	PP[3][0] -= K[1][1] * t[1][0];
	PP[3][1] -= K[1][1] * t[1][1];
		
	Angle[1]+= K[1][0] * Angle_err[1];	 //������� Y��ƫ���
	Q_bias[1]+= K[1][1] * Angle_err[1];	 //�������
	Gyro_x[1]= GYRO_X - Q_bias[1];	 //���ֵ(�������)��΢��=���ٶ�	

       
} 

 
 //�ú�����Ŀ����Ҫ�Ǻ�ʵ������P ����ļ��㷽ʽ
//ʵ��֤����Pk+1 = APk + PkA'+Q ���ַ�ʽ�����ǲ��Ե�
//
// crp
// 2016 01 09  
void Kalman_Filter_Verify(float GYRO_Y,float Acc_angl_x)//Kalman�˲���100MHz�Ĵ���ʱ��Լ****ms��	
{     
  Angle[0]+=(GYRO_Y - Q_bias[0])*dt;           //�������
	
//	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt + PP[1][1]* dt* dt + Q_angle; // Pk+1 = APkA'+Q
	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt  + Q_angle; // �����Ժ��Ե� PP[1][1]* dt* dt �� ��Ϊ��̫С�ˣ�
	PP[0][1] = PP[0][1] - PP[1][1] * dt;//�÷�����̬Ч��������  �Ƽ�ʹ�� 
	PP[1][0] = PP[1][0] - PP[1][1] * dt;// Q ��һ���Խ���
	PP[1][1] = PP[1][1] + Q_gyro;
	
//	PP[0][0] = PP[0][0]*2 - PP[0][1]* dt - PP[1][0]* dt + Q_angle; // Pk+1 = APk + PkA'+Q
//	PP[0][1] = PP[0][1]*2 - PP[1][1] * dt;//ʵ�ʼ��Ч������   
//	PP[1][0] = PP[1][0]*2 - PP[1][1] * dt;// Q ��һ���Խ���
//	PP[1][1] = PP[1][1]*2 + Q_gyro;	
	
//	PP[0][0] = PP[0][0] - PP[0][1]* dt - PP[1][0]* dt + Q_angle* dt; // ʹ��ʱ��ļ��㷽ʽ Ч������
//	PP[0][1] = PP[0][1] - PP[1][1] * dt;   
//	PP[1][0] = PP[1][0] - PP[1][1] * dt;// Q ��һ���Խ���
//	PP[1][1] = PP[1][1] + Q_gyro* dt;		

  
	Angle_err[0]=Acc_angl_x - Angle[0];	//�۲�ֵ

	K[0][0] = C[0] * PP[0][0]/(R_angle + PP[0][0]);//���㿨��������
	K[0][1] = C[0] * PP[0][1]/(R_angle + PP[0][0]);
	
	Angle[0]+= K[0][0] * Angle_err[0];	 //�������
	Q_bias[0]+= K[0][1] * Angle_err[0];	 //�������
	Gyro_x[0]= GYRO_Y - Q_bias[0];	 //���ֵ(�������)��΢��=���ٶ�	
	

	PP[1][0] = PP[1][0] - K[0][1] * PP[0][0];//����������Э����
	PP[1][1] = PP[1][1] - K[0][1] * PP[0][1];
	PP[0][0] = PP[0][0] * (1-K[0][0]);		 
	PP[0][1] = PP[0][1] * (1-K[0][0]);
 
	
} 
