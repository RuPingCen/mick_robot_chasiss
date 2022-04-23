#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h" 

#include "math.h"
#include "IO_IIC.h"
#include "MPU6050.h" 
#include "HMC5883.h"
#include "MPU9250.h"
#include "bsp_uart.h" 

#include "IMU.h"

//#include "Scope_API.h" //����ʾ����
#include "Seven_Lab_MiniIMU.h" //����ʾ����

//ʹ��֮ǰ��Ҫ���������жԴ���ģ���ʼ��  ����ʹ���� UART1 ģ��

extern float OutData[4];
 

#define Rad_to_Angle						57.324841    //���ȵ��Ƕ�
#define Angle_to_Rad   	        0.0174533    //�ȵ��Ƕ�

 
imu_Dat IMU_Data;//IMU���ݽṹ��
 
// int16_t GYRO_OFFSET[3]={53,-29,-6}; 
int16_t imu_tem[6]; 
int32_t imu_Groy_tem[3];
static char imu_timers=1;

void IMU_Preper_Data(void)
{
 
	if(imu_timers == 0) //��һ�ν��� ���ڱ궨������
	{
		int64_t imu_Groy_tem[3];
		char i=0;		 
		imu_Groy_tem[0]=0;
		imu_Groy_tem[1]=0;
		imu_Groy_tem[2]=0;
		for(i=0;i<50;i++)
		{
				MPU6050_Data_Process(imu_tem); //��ȡ9250����  
				imu_Groy_tem[0]+=imu_tem[3];
				imu_Groy_tem[1]+=imu_tem[4];
				imu_Groy_tem[2]+=imu_tem[5];
				Multiple_read_HMC5883(IMU_Data.magADC);//��βɼ����ڸ���FIFO
		}
	 
		 IMU_Data.gyroOffset[0]=(imu_Groy_tem[0]+45)/50;
		 IMU_Data.gyroOffset[1]=(imu_Groy_tem[1]+45)/50;
		 IMU_Data.gyroOffset[2]=(imu_Groy_tem[2]+45)/50;
		
		imu_timers++;
	}
	MPU6050_Data_Process(imu_tem); //��ȡ9250����  

	IMU_Data.accADC[0]=imu_tem[0]; 
	IMU_Data.accADC[1]=imu_tem[1]; 
	IMU_Data.accADC[2]=imu_tem[2]; 
	
	IMU_Data.gyroADC[0]=imu_tem[3]-IMU_Data.gyroOffset[0];
	IMU_Data.gyroADC[1]=imu_tem[4]-IMU_Data.gyroOffset[1];
	IMU_Data.gyroADC[2]=imu_tem[5]-IMU_Data.gyroOffset[2];

	IMU_Data.gyroRaw[0]=IMU_Data.gyroADC[0]*Angle_to_Rad/32.8f;
	IMU_Data.gyroRaw[1]=IMU_Data.gyroADC[1]*Angle_to_Rad/32.8f;
	IMU_Data.gyroRaw[2]=IMU_Data.gyroADC[2]*Angle_to_Rad/32.8f;

	Multiple_read_HMC5883(IMU_Data.magADC);
 
//	UART_send_intdata(USART1,IMU_Data.accADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.accADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.accADC[2]); UART_send_char(USART1,'\t');

//	UART_send_intdata(USART1,IMU_Data.gyroADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.gyroADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.gyroADC[2]); UART_send_char(USART1,'\t');

//	UART_send_intdata(USART1,IMU_Data.magADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.magADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.magADC[2]); UART_send_char(USART1,'\n');
}
 
 
 
 
 
 /**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

 
 
 
/**************************ʵ�ֺ���********************************************
*
*����ԭ��:	   void IMU_AHRSupdate
*
*��������:	  ����AHRS ������Ԫ�� 
*
*��������� ��ǰ�Ĳ���ֵ��
*
*���������  ��
*
*��ע���ο� crazepony   
*       20150503   (CRP) 
*******************************************************************************/
#define Kp      15.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki       0.01f   // integral gain governs rate of convergence of gyroscope biases
#define halfT    0.005f   // integral gain governs rate of convergence of gyroscope biases

 

volatile float exInt=0.0f, eyInt=0.0f, ezInt=0.0f;  // ������
volatile float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f; // ȫ����Ԫ��
volatile float q[4]; //����Ԫ���ݴ����


void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�ѼӼƵ���ά����ת�ɵ�λ������

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  //�ӵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ��  Ȼ���Y����ķ����궨Ϊ0
	
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);//����Ϊ�˼���������  ��Ϊq1��q2��q3��q4ƽ�����ǵ���1 ��
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
  /*
  �����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
  ��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣�������by=0��bx=ĳֵ
  ������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
  �����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
  �ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
  ��Ϊby=0�����Ծͼ򻯳�(bx*bx)  = ((hx*hx) + (hy*hy))�������bx��
  */         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  /**************************************************************************************
     �����������ת������ԣ�0,0��g���Ժ�õ����������ǻ�������ϵ�µ�������������������Ϊ������һ����λҲ���ǣ�0,0,1����
		 ������C*(0,0,1) ,Ҳ���Ǿ����ұ���һ�С�
  **************************************************************************************/
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	
  /**************************************************************************************
  ���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
  ��Ϊby=0�����������漰��by�Ĳ��ֶ���ʡ���ˡ�
  ������������vxyz�����㣬��Ϊ����g��gz=1��gx=gy=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
  *************************************************************************************/
	
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  //���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  /************************************************************************************************************
  axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
  axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
  ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
  ������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
  �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ�
	��������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
  ************************************************************************************************************/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
			exInt = exInt + ex * Ki * halfT;
			eyInt = eyInt + ey * Ki * halfT;
			ezInt = ezInt + ez * Ki * halfT;

			
			gx = gx + Kp*ex + exInt;// �ò���������PI����������ƫ
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;

  }

 /******************************************************************************************************************************************************  
	   ��Ԫ��΢�ַ���
     ��Ԫ��΢�ַ��̱���ֻ�ǻ��ڽ��ٶȵģ�Ҳ����˵����֪�ϸ����ڵ���̬���ͱ��β����õ��Ľ��ٶȣ��õ������ڵ���̬��
	   �ڽ��ٶ��������������������̬���𵽼�С�������ã�������һ������Ҫ�ĵط�.
	
ע��ex ey ez��ʾ���Ǽ��ٶȼƵ�ֵ���Ԥ��ֵ��Ҳ���ǲ���ֵA������������ʵ�ʻ�������ϵ�µ���������ת��Ԥ��ֵB������������������Ϊ�Ļ�������ϵ�µ���������һ��������
    Ҳ����B-A��Ȼ�������Ǽ������������������Ҫ��������Ϊ�Ļ���������ת��ʵ�ʻ������꣬Ϊʲô���Ǽ���A-B�أ��ǲ��Ǹ���ˣ������ֵ��˼����?
	
	  ����˵���������������Ҫ˳ʱ����ת�����Ǿ���������ʱ����תһ���ĽǶȡ�ͨ��һ�㽲������������Ϊ�Ļ����������ʵ�ʻ������껹������ôЩ��
	  ��Ҫ����Ԫ��΢�ַ�����Ľ��ٶ���������ƫ��.�����������ǵش��ˣ����Ƶģ�������ֵ����һ���ǲ���ֵA���ش�������ʵ�ʻ�������ϵ�µ�������
	  ��ת��Ԥ��ֵB���ش�������������Ϊ�Ļ�������ϵ�µ�������.A���ǵشŴ�����������ֵ��B��������м����ˣ�����֪�������ڵ�������ϵ�������ǣ�0,0,1����
	  ���ǵش����ǲ������������ֻ�ܼ����������ϵx�᷽�򳯱��������Ļ�y����ش���0���������ǲ�֪��x��z�����Ƕ���
		�㷨����ô���ģ�����������Ϊ�Ļ�������ϵ�ǶԵģ���ô�����õ��ĵش�����ת������������ϵ��ȥ�������õ�һ��������
	  ������Ϊ��������ϵ�£�z������Ǹ�����z����ķ�����y������0��x�����Ǹ�����xyƽ���ϵ�ͶӰ�������������¾ͺʹ�����������һ���ˡ�
	  ������������Ȱѵشű任����������ϵ��Ȼ����һ����y��شű��0��Ȼ���ڱ任�ػ�������ϵ���ٺ�ʵ�ʲ����ĵش�����
*******************************************************************************************************************************************************/
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // ��Ԫ���淶��
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;

	IMU_Data.q[0] = q0; //���ص�ǰֵ
	IMU_Data.q[1] = q1;
	IMU_Data.q[2] = q2;
	IMU_Data.q[3] = q3;

	IMU_Data.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* Rad_to_Angle; // yaw
	IMU_Data.pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2)* Rad_to_Angle; // pitch
	IMU_Data.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* Rad_to_Angle; // roll
	
}



void MPU6050_Routing(void)
{
	IMU_Preper_Data();
	// GPIO_SetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������1��

	IMU_update(IMU_Data.gyroRaw[0],IMU_Data.gyroRaw[1],IMU_Data.gyroRaw[2],
					IMU_Data.accADC[0], IMU_Data.accADC[1], IMU_Data.accADC[2],
					IMU_Data.magADC[0], IMU_Data.magADC[1], IMU_Data.magADC[2]);

	// GPIO_ResetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������0��

	//OutData[0]=IMU_Data.pitch;
	//OutData[1]=IMU_Data.roll;
	//OutData[2]=IMU_Data.yaw;
	//OutPut_Data();

}

void MPU9250_Routing(void)
{
	MPU9250_READ_ACCEL(IMU_Data.accADC);
	MPU9250_READ_GYRO(IMU_Data.gyroADC);  
	MPU9250_READ_MAG(IMU_Data.magADC);
	
	if(imu_timers <= 50) //��һ�ν��� ���ڱ궨������
	{
		if(imu_timers == 0)
		{
			imu_Groy_tem[0]=0;
			imu_Groy_tem[1]=0;
			imu_Groy_tem[2]=0;
		}
 
		if((IMU_Data.gyroADC[0] > -400 && IMU_Data.gyroADC[0]<400) || 
			(IMU_Data.gyroADC[1] > -400 && IMU_Data.gyroADC[1]<400) ||
			(IMU_Data.gyroADC[2] > -400 && IMU_Data.gyroADC[2]<400))
		{
			imu_Groy_tem[0]+=IMU_Data.gyroADC[0];
			imu_Groy_tem[1]+=IMU_Data.gyroADC[1];
			imu_Groy_tem[2]+=IMU_Data.gyroADC[2];
			
			imu_timers++;
		}
				  
		if(imu_timers == 50)
		{
			IMU_Data.gyroOffset[0]=(imu_Groy_tem[0]+45)/50;
			IMU_Data.gyroOffset[1]=(imu_Groy_tem[1]+45)/50;
			IMU_Data.gyroOffset[2]=(imu_Groy_tem[2]+45)/50;
			
//			UART_send_string(USART2,"Gyro offset value:");
//			UART_send_intdata(USART2,IMU_Data.gyroOffset[0]); UART_send_char(USART2,'\t');
//			UART_send_intdata(USART2,IMU_Data.gyroOffset[1]); UART_send_char(USART2,'\t');
//			UART_send_intdata(USART2,IMU_Data.gyroOffset[2]); UART_send_char(USART2,'\n');
			
			imu_timers++;
		}
	}
	
	IMU_Data.gyroADC[0]=IMU_Data.gyroADC[0]-IMU_Data.gyroOffset[0];
	IMU_Data.gyroADC[1]=IMU_Data.gyroADC[1]-IMU_Data.gyroOffset[1];
	IMU_Data.gyroADC[2]=IMU_Data.gyroADC[2]-IMU_Data.gyroOffset[2];
	
	IMU_Data.gyroRaw[0]=IMU_Data.gyroADC[0]*Angle_to_Rad/32.8f;
	IMU_Data.gyroRaw[1]=IMU_Data.gyroADC[1]*Angle_to_Rad/32.8f;
	IMU_Data.gyroRaw[2]=IMU_Data.gyroADC[2]*Angle_to_Rad/32.8f;
	
	if((IMU_Data.magADC[0] == -1) && (IMU_Data.magADC[1] == -1) && (IMU_Data.magADC[2] == -1))
	{
		;
	}
	else
	{
			IMU_update(IMU_Data.gyroRaw[0],IMU_Data.gyroRaw[1],IMU_Data.gyroRaw[2],
					IMU_Data.accADC[0], IMU_Data.accADC[1], IMU_Data.accADC[2],
					IMU_Data.magADC[0], IMU_Data.magADC[1], IMU_Data.magADC[2]);
	}
}
void IMU_Routing(void)
{
	//MPU6050_Routing();
	//GPIO_SetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������1��
		MPU9250_Routing();
	//GPIO_ResetBits(GPIOF,GPIO_Pin_7);//��λһ������  ʹ���������0��
	
//	UART_send_intdata(USART1,IMU_Data.accADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.accADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.accADC[2]); UART_send_char(USART1,'\t');

//	UART_send_intdata(USART1,IMU_Data.gyroADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.gyroADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.gyroADC[2]); UART_send_char(USART1,'\t');

//	UART_send_intdata(USART1,IMU_Data.magADC[0]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.magADC[1]); UART_send_char(USART1,'\t');
//	UART_send_intdata(USART1,IMU_Data.magADC[2]); UART_send_char(USART1,'\n');
	
//	UART_send_floatdat(USART1,IMU_Data.pitch); UART_send_char(USART1,'\t');
//	UART_send_floatdat(USART1,IMU_Data.roll); UART_send_char(USART1,'\t');
//	UART_send_floatdat(USART1,IMU_Data.yaw); UART_send_char(USART1,'\n');
	
	
}
// ���� ros����IMU����
void IMU_Upload_Message(void)
{
		static  uint32_t IMU_upload_counter=0;
		unsigned char senddat[35];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	

		int16_t pitch,roll,yaw;

		pitch =(IMU_Data.pitch*100+0.5)/1;
		roll=(IMU_Data.roll*100+0.5)/1;
		yaw=(IMU_Data.yaw*100+0.5)/1;
		
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//���ݳ����ں��渳ֵ
		senddat[i++]=0xA0; //����λ 0xA0
	
	  //�ϴ�����֡����
		senddat[i++]=(IMU_upload_counter>>24);
		senddat[i++]=(IMU_upload_counter>>16);
		senddat[i++]=(IMU_upload_counter>>8);
		senddat[i++]=(IMU_upload_counter);
			
 
		senddat[i++] = (uint8_t)(IMU_Data.accADC[0] >> 8); //int16
		senddat[i++] = (uint8_t)(IMU_Data.accADC[0]);
		senddat[i++] = (IMU_Data.accADC[1] >> 8);  
		senddat[i++] = (IMU_Data.accADC[1]);
		senddat[i++] = (IMU_Data.accADC[2] >> 8);  
		senddat[i++] = (IMU_Data.accADC[2]);
		senddat[i++] = (IMU_Data.gyroADC[0]&0xff00 >> 8);  
		senddat[i++] = (IMU_Data.gyroADC[0]&0x00ff);
		senddat[i++] = (uint8_t)(IMU_Data.gyroADC[1]&0xff00 >> 8); 
		senddat[i++] = (uint8_t)(IMU_Data.gyroADC[1]&0x00ff);
		senddat[i++] = (uint8_t)(IMU_Data.gyroADC[2]&0xff00 >> 8);  
		senddat[i++] = (uint8_t)(IMU_Data.gyroADC[2]&0x00ff);
		senddat[i++] = (uint8_t)(IMU_Data.magADC[0]&0xff00 >> 8);  
		senddat[i++] = (uint8_t)(IMU_Data.magADC[0]&0x00ff);
		senddat[i++] = (uint8_t)(IMU_Data.magADC[1]&0xff00 >> 8); 
		senddat[i++] = (uint8_t)(IMU_Data.magADC[1]&0x00ff);
		senddat[i++] = (uint8_t)(IMU_Data.magADC[2]&0xff00 >> 8);  
		senddat[i++] = (uint8_t)(IMU_Data.magADC[2]&0x00ff);
		senddat[i++] = pitch>>8; 
		senddat[i++] = pitch;
		senddat[i++] = roll>>8; 
		senddat[i++] = roll;
		senddat[i++] = yaw>>8; 
		senddat[i++] = yaw;

		senddat[2]=i-1; //���ݳ���
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		IMU_upload_counter++;
}
