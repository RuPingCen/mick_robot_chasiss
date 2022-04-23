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

//#include "Scope_API.h" //虚拟示波器
#include "Seven_Lab_MiniIMU.h" //虚拟示波器

//使用之前需要在主函数中对串口模块初始化  这里使用了 UART1 模块

extern float OutData[4];
 

#define Rad_to_Angle						57.324841    //弧度到角度
#define Angle_to_Rad   	        0.0174533    //度到角度

 
imu_Dat IMU_Data;//IMU数据结构体
 
// int16_t GYRO_OFFSET[3]={53,-29,-6}; 
int16_t imu_tem[6]; 
int32_t imu_Groy_tem[3];
static char imu_timers=1;

void IMU_Preper_Data(void)
{
 
	if(imu_timers == 0) //第一次进入 用于标定陀螺仪
	{
		int64_t imu_Groy_tem[3];
		char i=0;		 
		imu_Groy_tem[0]=0;
		imu_Groy_tem[1]=0;
		imu_Groy_tem[2]=0;
		for(i=0;i<50;i++)
		{
				MPU6050_Data_Process(imu_tem); //读取9250数据  
				imu_Groy_tem[0]+=imu_tem[3];
				imu_Groy_tem[1]+=imu_tem[4];
				imu_Groy_tem[2]+=imu_tem[5];
				Multiple_read_HMC5883(IMU_Data.magADC);//多次采集用于更新FIFO
		}
	 
		 IMU_Data.gyroOffset[0]=(imu_Groy_tem[0]+45)/50;
		 IMU_Data.gyroOffset[1]=(imu_Groy_tem[1]+45)/50;
		 IMU_Data.gyroOffset[2]=(imu_Groy_tem[2]+45)/50;
		
		imu_timers++;
	}
	MPU6050_Data_Process(imu_tem); //读取9250数据  

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
 
 
 
 
 
 /**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
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

 
 
 
/**************************实现函数********************************************
*
*函数原型:	   void IMU_AHRSupdate
*
*功　　能:	  更新AHRS 更新四元数 
*
*输入参数： 当前的测量值。
*
*输出参数：  无
*
*备注：参考 crazepony   
*       20150503   (CRP) 
*******************************************************************************/
#define Kp      15.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki       0.01f   // integral gain governs rate of convergence of gyroscope biases
#define halfT    0.005f   // integral gain governs rate of convergence of gyroscope biases

 

volatile float exInt=0.0f, eyInt=0.0f, ezInt=0.0f;  // 误差积分
volatile float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f; // 全局四元数
volatile float q[4]; //　四元数暂存变量


void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;

  // 先把这些用得到的值算好
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
  //把加计的三维向量转成单位向量。

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  //从电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）  然后对Y方向的分量标定为0
	
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);//这里为了减少运算量  因为q1、q2、q3、q4平方和是等于1 的
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
  /*
  计算地理坐标系下的磁场矢量bxyz（参考值）。
  因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
  但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
  我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
  磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
  因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
  */         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  /**************************************************************************************
     这个矩阵是旋转矩阵乘以（0,0，g）以后得到的向量，是机体坐标系下的重力的向量，我们认为重力是一个单位也就是（0,0,1），
		 所以是C*(0,0,1) ,也就是矩阵右边那一列。
  **************************************************************************************/
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	
  /**************************************************************************************
  我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
  因为by=0，所以所有涉及到by的部分都被省略了。
  类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
  *************************************************************************************/
	
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  /************************************************************************************************************
  axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
  那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
	（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  ************************************************************************************************************/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
			exInt = exInt + ex * Ki * halfT;
			eyInt = eyInt + ey * Ki * halfT;
			ezInt = ezInt + ez * Ki * halfT;

			
			gx = gx + Kp*ex + exInt;// 用叉积误差来做PI修正陀螺零偏
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;

  }

 /******************************************************************************************************************************************************  
	   四元数微分方程
     四元数微分方程本来只是基于角速度的，也就是说，已知上个周期的姿态，和本次测量得到的角速度，得到本周期的姿态，
	   在角速度里加入误差反馈，来调节姿态，起到减小误差的作用，这里有一个很重要的地方.
	
注：ex ey ez表示的是加速度计的值叉乘预测值，也就是测量值A（重力向量在实际机体坐标系下的向量）旋转到预测值B（重力向量在我们认为的机体坐标系下的向量）的一个向量，
    也就是B-A，然后陀螺仪加上这个向量，我们需要把我们认为的机体坐标旋转到实际机体坐标，为什么不是加上A-B呢，是不是搞错了？这里很值得思考下?
	
	  简单来说，就是如果向量需要顺时针旋转，我们就让坐标逆时针旋转一样的角度。通俗一点讲，就是我们认为的机体坐标距离实际机体坐标还差了那么些，
	  就要在四元数微分方程里的角速度里，加入这个偏差.接下来，就是地磁了，类似的，有两个值，第一个是测量值A（地磁向量在实际机体坐标系下的向量）
	  旋转到预测值B（地磁向量在我们认为的机体坐标系下的向量）.A就是地磁传感器测量的值，B呢这里就有技巧了，我们知道重力在地面坐标系下向量是（0,0,1），
	  可是地磁我们并不清楚，我们只能假设地面坐标系x轴方向朝北，这样的话y方向地磁是0，可是我们不知道x和z具体是多少
		算法是这么做的，假设我们认为的机体坐标系是对的，那么测量得到的地磁向量转换到地面坐标系上去，这样得到一个向量，
	  所以认为地面坐标系下，z方向就是该向量z方向的分量，y方向是0，x方向是该向量xy平面上的投影长。接下来的事就和处理重力向量一样了。
	  代码里面就是先把地磁变换到地面坐标系，然后处理一下让y轴地磁变成0，然后在变换回机体坐标系，再和实际测量的地磁求误差。
*******************************************************************************************************************************************************/
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;

	IMU_Data.q[0] = q0; //返回当前值
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
	// GPIO_SetBits(GPIOF,GPIO_Pin_7);//置位一个引脚  使引脚输出‘1’

	IMU_update(IMU_Data.gyroRaw[0],IMU_Data.gyroRaw[1],IMU_Data.gyroRaw[2],
					IMU_Data.accADC[0], IMU_Data.accADC[1], IMU_Data.accADC[2],
					IMU_Data.magADC[0], IMU_Data.magADC[1], IMU_Data.magADC[2]);

	// GPIO_ResetBits(GPIOF,GPIO_Pin_7);//复位一个引脚  使引脚输出‘0’

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
	
	if(imu_timers <= 50) //第一次进入 用于标定陀螺仪
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
	//GPIO_SetBits(GPIOF,GPIO_Pin_7);//置位一个引脚  使引脚输出‘1’
		MPU9250_Routing();
	//GPIO_ResetBits(GPIOF,GPIO_Pin_7);//复位一个引脚  使引脚输出‘0’
	
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
// 测试 ros接收IMU数据
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
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0xA0; //命令位 0xA0
	
	  //上传数据帧计数
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

		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		IMU_upload_counter++;
}
