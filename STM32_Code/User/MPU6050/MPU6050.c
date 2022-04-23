#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h" 
#include "stm32f10x_Delay.h"
 
#include "IO_IIC.h"
#include "MPU6050.h"
 


	
	
#define  FIFO_Length   5 //���������˲�����
 
 
//static int16_t ACC_OFFSET[3]={ 240,-160,14350};      // ���ٶȼ� X��Y��Z�� ƫ��ֵ
//static int16_t GYRO_OFFSET[3]={53,-29,-6};   				// ������ X��Y��Z�� ƫ��ֵ

int16_t MPU6050_FIFO[6][FIFO_Length+1]={'0'};





void mpu6050_delay(unsigned int temp)
{
   while(temp-- >0);
}
/*************************************************************************
*  �������ƣ�MPU6050_Init
*  ����˵����MPU6050��ʼ�� �ڲ��ѳ�ʼ��IICģ��
*  ����˵����I2Cn        ģ��ţ�I2C0��I2C1��
*            doub_MPU6050dat    Double������
*  �������أ���
*  �޸�ʱ�䣺2014-7-12
*  ��    ע��CRP
*************************************************************************/
#ifdef  Hadware_IIC 

void MPU6050_Init(I2C_TypeDef* I2Cx)//MPU6050��ʼ�� �� 2000 ��/s �� 4g  5hz
{
	 I2C_STM32_Config_Init(I2Cx,400000);
		mpu6050_delay(72000); 
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,MPU6050_clock_PLL_X_gyro);//ʹ���ڲ�����ʱ��8Mhz
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_2,MPU6050_wakeupfrequence_10hz);// ���û���Ƶ��10 hz
	 //I2C_WriteAddr(i2cn, MPU6050_SlaveAddress,MPU6050_RA_SMPLRT_DIV,0x07);//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_CONFIG,MPU6050_DLPF_BANDWIDTH_5hz);
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_Range_2000);//�� 2000 ��/s 
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_ACCEL_CONFIG,MPU6050_ACCEL_Range_4g|MPU6050_ACCEL_DHPF_5HZ);//�� 4g  5hz
}
#else 
void MPU6050_Init(void)//MPU6050��ʼ�� �� 2000 ��/s �� 2g  5hz
{
	// unsigned char dev_ID=0;
	 IO_IIC_GPIO_Init();//IIC���ų�ʼ��     
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,0x80);//MPU6050��λ
	 Delay_10us(2000);// nTime*100us ��ʱ����
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,MPU6050_CLOCK_PLL_ZGYRO);//MPU6050��ʼ�� ʹ��GYRO Z
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_SMPLRT_DIV,0x00);
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_CONFIG,MPU6050_DLPF_BW_42);//���Ƶ��1K   ����42hz
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_FS_2000);//�� 2000 ��/s 
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_ACCEL_CONFIG,MPU6050_DHPF_5 | MPU6050_ACCEL_FS_2g);//�� 2g  5hz
	/*
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_INT_PIN_CFG,0x30);// ����6050�����ж�����   
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_INT_ENABLE,0x01);// ������׼�����Ժ����int ��������� 50us �ĸߵ�ƽ ��һֱ������������������ɣ�
	*/
	 while(IO_IIC_read_Data(MPU6050_SlaveAddress, MPU6050_RA_WHO_AM_I) != 0x68);//���MPU6050 �Ƿ��Ѿ�����
}
#endif 

 
/*************************************************************************
*  �������ƣ�MPU6050_Data_Process
*  ����˵����MPU6050���ݶ�ȡ ��������
*  ����˵����I2Cn        ģ��ţ�I2C0��I2C1��
*            doub_MPU6050dat    Double������
*  �������أ���
*  �޸�ʱ�䣺2014-7-12
*  ��    ע��CRP
*************************************************************************/
#ifdef  Hadware_IIC 
	
void MPU6050_Data_Process(I2C_TypeDef* I2Cx,short int *u16MPU6050dat)//MPU6050���ݶ�ȡ ��������
{
   /*u16MPU6050dat[0]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0])*2/4096.0;//�� 2����Ϊ�ڳ�ʼ����ʱ��ѡ�ŵ�����Ϊ ��2g
     u16MPU6050dat[1]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_YOUT_H)-ACC_OFFSET[1])*2/4096.0;//����������Ӧ���������ʾ��һ��������
     u16MPU6050dat[2]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2])*2/4096.0;
     u16MPU6050dat[3]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0])/16.4;
     u16MPU6050dat[4]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_YOUT_H)-GYRO_OFFSET[1])/16.4;
     u16MPU6050dat[5]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_ZOUT_H)-GYRO_OFFSET[2])/16.4;      
   */
       
     u16MPU6050dat[0]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0]) ;//�� 2����Ϊ�ڳ�ʼ����ʱ��ѡ�ŵ�����Ϊ ��2g
     u16MPU6050dat[1]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_YOUT_H)+ACC_OFFSET[1]) ;//����������Ӧ���������ʾ��һ��������
     u16MPU6050dat[2]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2]) ;
     u16MPU6050dat[3]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0]) ;
     u16MPU6050dat[4]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_YOUT_H)+GYRO_OFFSET[1]) ;
     u16MPU6050dat[5]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_ZOUT_H)+GYRO_OFFSET[2]) ;
     
}
#else
void MPU6050_Data_Process(int16_t *int16t_MPU6050dat)//MPU6050���ݶ�ȡ ��������
{
	    
	  
//			int16t_MPU6050dat[0]=MPU6050_GetData(MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0];//ʹ����Ԫ�� ���ٶ����� ����Ҫ��ȥOFFSET 
//			int16t_MPU6050dat[1]=MPU6050_GetData(MPU6050_RA_ACCEL_YOUT_H)-ACC_OFFSET[1];// 
//			int16t_MPU6050dat[2]=MPU6050_GetData(MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2];
//			int16t_MPU6050dat[3]=MPU6050_GetData(MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0];
//			int16t_MPU6050dat[4]=MPU6050_GetData(MPU6050_RA_GYRO_YOUT_H)-GYRO_OFFSET[1];
//			int16t_MPU6050dat[5]=MPU6050_GetData(MPU6050_RA_GYRO_ZOUT_H)-GYRO_OFFSET[2];  
	
	
	   	int16t_MPU6050dat[0]=MPU6050_GetData(MPU6050_RA_ACCEL_XOUT_H);//ʹ����Ԫ�� ���ٶ����� ����Ҫ��ȥOFFSET 
			int16t_MPU6050dat[1]=MPU6050_GetData(MPU6050_RA_ACCEL_YOUT_H); 
			int16t_MPU6050dat[2]=MPU6050_GetData(MPU6050_RA_ACCEL_ZOUT_H); 
			int16t_MPU6050dat[3]=MPU6050_GetData(MPU6050_RA_GYRO_XOUT_H);//������������IMU �����ȥ��OFFSET 
			int16t_MPU6050dat[4]=MPU6050_GetData(MPU6050_RA_GYRO_YOUT_H);
			int16t_MPU6050dat[5]=MPU6050_GetData(MPU6050_RA_GYRO_ZOUT_H); 


			MPU6050_newValues(int16t_MPU6050dat[0],int16t_MPU6050dat[1],int16t_MPU6050dat[2],//���������˲�
											  int16t_MPU6050dat[3],int16t_MPU6050dat[4],int16t_MPU6050dat[5]);


			int16t_MPU6050dat[0]=MPU6050_FIFO[0][FIFO_Length];
			int16t_MPU6050dat[1]=MPU6050_FIFO[1][FIFO_Length];
			int16t_MPU6050dat[2]=MPU6050_FIFO[2][FIFO_Length];
			int16t_MPU6050dat[3]=MPU6050_FIFO[3][FIFO_Length];
			int16t_MPU6050dat[4]=MPU6050_FIFO[4][FIFO_Length];
			int16t_MPU6050dat[5]=MPU6050_FIFO[5][FIFO_Length];

	
//			UART_send_intdata(USART1,int16t_MPU6050dat[0]);UART_send_char(USART1,'\t'); 
//			UART_send_intdata(USART1,int16t_MPU6050dat[1]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[2]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[3]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[4]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[5]);UART_send_char(USART1,'\n');
			 

 
}
#endif

/*************************************************************************
*  �������ƣ�MPU_GetData
*  ����˵�����ϳ�����
*  ����˵����I2Cn        ģ��ţ�I2C0��I2C1��
*  �������أ�short int ��
*  �޸�ʱ�䣺2014-7-12
*  ��    ע��CRP
*************************************************************************/
#ifdef  Hadware_IIC 
short int MPU6050_GetData(I2C_TypeDef* I2Cx,unsigned char REG_Address) 
{
	unsigned char H,L; 
	H=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,REG_Address);            //��ȡ�߰�λ         
	L=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,REG_Address+1);          //��ȡ�Ͱ�λ
	return ((H<<8)+L);                                    //�ϳ�����
}  

#else
short int MPU6050_GetData(unsigned char REG_Address) 
{
	unsigned char H,L;
	H=IO_IIC_read_Data(MPU6050_SlaveAddress,REG_Address);            //��ȡ�߰�λ
	L=IO_IIC_read_Data(MPU6050_SlaveAddress,REG_Address+1);          //��ȡ�Ͱ�λ
      
	return ((H<<8)+L);                                    //�ϳ�����
}   
#endif




/**************************����6050 ����ʱ��********************************************
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(I2C_TypeDef* I2Cx,unsigned char source)
{
	 
	I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,source);//�� 2g  5hz
        
}
/**************************��������********************************************
  �������ܣ�����6050�����ɣɣ��룶���������ӿڣɣɣ�������
 ��
*******************************************************************************/
/*
void MPU6050_setI2CBypassEnabled(I2Cn i2cn,unsigned char  enabled) 
{
	��
        I2C_WriteAddr(i2cn, MPU6050_SlaveAddress,PWR_MGMT_1,enabled);//sleep λ���� 0
}
 */

/**************************��������********************************************
 �������ܣ�����MPU6050 ����ģʽ
 enabled =1   ˯��
 enabled =0   ��������
*******************************************************************************/
#ifdef  Hadware_IIC 
void MPU6050_setSleepEnabled(I2C_TypeDef* I2Cx,unsigned char  enabled) 
{  
    unsigned char dat;
    dat=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,PWR_MGMT_1);//�ѼĴ���  PWR_MGMT_1�����ݶ�ȡ����
    if(enabled==1)//˯��ģʽ
    {
        dat |=0x40; 
        I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,PWR_MGMT_1,dat);//sleep λ���� 1
    } 
    else//��������ģʽ
    {
       dat &=0xBF; 
       I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,PWR_MGMT_1,dat);//sleep λ���� 0
    }
}
#else
void MPU6050_setSleepEnabled(uint8_t enabled) 
{
	  unsigned char tem;
	
   tem= IO_IIC_read_Data(MPU6050_SlaveAddress, MPU6050_RA_PWR_MGMT_1);
	
	 if(enabled ==1)
		  tem |=0x40;
	 else 
		  tem |=0x20;
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,tem); 
}
#endif



/*************************************************************************
*  �������ƣ�READ_temp
*  ����˵������ȡ6050�е��¶���ʾ�¶�
*  ����˵����I2Cn        ģ��ţ�I2C0��I2C1��
*  �������أ�short int ��
*  �޸�ʱ�䣺2014-7-12
*  ��    ע��CRP
*************************************************************************/
#ifdef  Hadware_IIC 
short int READ_temp(I2C_TypeDef* I2Cx)//��ʾ�¶�
 { 
        unsigned char Temp_h,Temp_l;
        short int Temperature;
        Temp_h=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,TEMP_OUT_H);            //��ȡ�¶ȸ߰�λ
        mpu6050_delay(30000); 
        Temp_l=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,TEMP_OUT_L);             //��ȡ�¶ȵͰ�λ
  	Temperature=Temp_h<<8|Temp_l;                               //�ϳ��¶�
  	Temperature = ((double)Temperature / 340.0) +36.25;   // ������¶�
  	return  Temperature ;
 }
 #else
 short int READ_temp(void)//��ʾ�¶�
{ 
		unsigned char Temp_h,Temp_l;
		short int Temperature;
		Temp_h=IO_IIC_read_Data(MPU6050_SlaveAddress,MPU6050_RA_TEMP_OUT_H);            //��ȡ�¶ȸ߰�λ

		Temp_l=IO_IIC_read_Data(MPU6050_SlaveAddress,MPU6050_RA_TEMP_OUT_L);             //��ȡ�¶ȵͰ�λ
		Temperature=Temp_h<<8|Temp_l;                               //�ϳ��¶�
		Temperature = ((double) (Temperature + 13200)) / 280;   // ������¶�
		return  Temperature ;
}
#endif  
 
 /**************************ʵ�ֺ���********************************************
*
*����ԭ��:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*
*��������:	    ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����
*
*��ע���ο�����ʵ����
*
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
		unsigned char i ;
		int64_t sum=0;
		for(i=1;i<FIFO_Length;i++)
		{	//FIFO ����
				MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
				MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
				MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
				MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
				MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
				MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[0][FIFO_Length-1]=ax;//���µ����ݷ��õ� ���ݵ������
		MPU6050_FIFO[1][FIFO_Length-1]=ay;
		MPU6050_FIFO[2][FIFO_Length-1]=az;
		MPU6050_FIFO[3][FIFO_Length-1]=gx;
		MPU6050_FIFO[4][FIFO_Length-1]=gy;
		MPU6050_FIFO[5][FIFO_Length-1]=gz;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{	//��ǰ����ĺϣ���ȡƽ��ֵ
			 sum+=MPU6050_FIFO[0][i];
		}
		MPU6050_FIFO[0][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[1][i];
		}
		MPU6050_FIFO[1][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[2][i];
		}
		MPU6050_FIFO[2][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[3][i];
		}
		MPU6050_FIFO[3][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[4][i];
		}
		MPU6050_FIFO[4][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[5][FIFO_Length]=sum/FIFO_Length;
}
