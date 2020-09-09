#include "stm32f10x.h"
#include "math.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "DJI_Motor.h"  


// C620电调控制范围-16384~0~16384 对应电调输出 -20A-0-20A 发送频率1KHz
// C620电调反馈 机械转子角度 0-360°  对应0-8191  转子速度为RPM单位  温度单位为℃
// 3508电机最大空载转速482  最大额定转速469 额定电流10A 最大连续转矩3 N*M
// 由于M3508电机配备了1:19的减速器 因此转子的最大转速为 482*19(RPM)

//大车四轮车使用的是10英寸的轮子   25.4cm
// 麦克纳姆轮的直径为 152.5mm



#define ABS(x)		((x>0)? (x): (-x)) 
#define MAX_WHEEL_SPEED 9158  //电机最大转速单位 RPM   482*19=9158


int set_v,set_spd[4]; 
uint32_t motor_upload_counter; //数据上传计数器

moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t moto_info;

pid_t pid_spd[4]; //四个电机对应的PID结构体

command_t recived_cmd; 


// pid_omg;
//pid_t pid_pos;
//pid_t pid_rol = {0};
//pid_t pid_pit = {0};
//pid_t pid_yaw = {0};
//pid_t pid_yaw_omg = {0};//角速度环
//pid_t pid_pit_omg = {0};//角速度环
//pid_t pid_yaw_alfa = {0};		//angle acce

//pid_t pid_chassis_angle={0};
//pid_t pid_poke = {0};
//pid_t pid_poke_omg = {0};
//pid_t pid_imu_tmp;
//pid_t pid_x;
//pid_t pid_cali_bby;
//pid_t pid_cali_bbp;


 // 初始化PID参数
void DJI_Motor_Init(void)
{
	unsigned char i=0;
	for(i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 5000,15.8f,	0.019f,	0.0f);  //4 motos angular rate closeloop.
	}
	set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = 0;//设置四个电机的目标速度
	
	motor_upload_counter=0;
}

// C620电调控制范围-16384~0~16384 对应电调输出 -20A-0-20A 发送频率1KHz
// C620电调反馈 机械转子角度 0-360°  对应0-8191  转子速度为RPM单位  温度单位为℃
//3508电机最大空载转速482  最大额定转速469 额定电流10A 最大连续转矩3 N*M
/**
* @funtion			大车四轮差速模型
* @Brief        根据设定的X方向的速度和航向角速度W 
*  							计算出四个轮子的速度
* @Param		speed_x 小车X方向的线速度(m/s)  speed_w小车旋转角速度(rad/s)
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w)
{
	float v1=0,v2=0,v3=0,v4=0;
	float L=0.68;//左右轮子的间距
	
  if((speed_x<-5) || (speed_x>5)) 
		speed_x=0;
  if((speed_w<-5) || (speed_w>5)) 
		speed_w=0;
 
	v1 = speed_x+speed_w*L;
	v4 = -(speed_x-speed_w*L);
	
	// 我们使用的是10英寸的轮子 直径为25.4cm 半径为12.7cm
  //60/(2*PI*R) = 75.191313 
	//乘以19是由于电机有1:19的减速比
	
	v1 =75.191313*v1*19;// from (m/s) to (RPM)
	v4 =75.191313*v4*19;
	
	v2 =v1; 
	v3 =v4;
	
	Mecanum_Wheel_Rpm_Model(v1,v2,v3,v4);
}

/**
* @funtion			麦克纳姆轮运动学模型(麦克纳姆轮呈‘米’字形排列)
* @Brief        将小车的速度分解成每个轮子的速度
* @Param		speed_x 小车X方向的线速度(m/s)  speed_w小车旋转角速度(rad/s)
* @Retval		None (该函数修改后还未测试)
* @Date     20190916
* @maker    crp
*/
void Mecanum_Wheel_Speed_Model(int16_t speed_x,int16_t speed_y,int16_t speed_w)
{
	float v1=0,v2=0,v3=0,v4=0;
	
	float K =0.36; // K=abs(Xn)+abs(Yn) Xn Yn表示麦轮的安装坐标
								//左右轮距 40cm 前后轮距36cm
//	float r_1 = 6.5574;//   6.5574= 1/0.1525 
	 
	if((speed_x<-5) || (speed_x>5)) 
		speed_x=0;
	if((speed_y<-5) || (speed_y>5)) 
		speed_x=0;
	if((speed_w<-5) || (speed_w>5)) 
		speed_w=0;	
	
	v1 =speed_x-speed_y-K*speed_w;
	v2 =speed_x+speed_y-K*speed_w;
	v3 =-(speed_x-speed_y+K*speed_w);
	v4 =-(speed_x+speed_y+K*speed_w);
	
	// 麦克纳姆轮的直径为 152.5mm     60/(2*PI*R) = 125.23668 
	//乘以19是由于电机有1:19的减速比
	
	v1 =125.23668*v1*19;// from (m/s) to (RPM)
	v2 =125.23668*v2*19;
	v3 =125.23668*v3*19;
	v4 =125.23668*v4*19;
	
	Mecanum_Wheel_Rpm_Model(v1,v2,v3,v4);
}



/**
* @funtion	 设置每个轮子PID控制器的目标值
* @Brief     输入参数单位（RPM）   
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{	 
	if(v1>MAX_WHEEL_SPEED) 		v1=MAX_WHEEL_SPEED;
	else if(v1< -MAX_WHEEL_SPEED)		v1=-MAX_WHEEL_SPEED;
	else ;
	
	if((v1>-5) && (v1<5))  v1=0;
	
	if(v2>MAX_WHEEL_SPEED) 		v2=MAX_WHEEL_SPEED;
	else if(v2< -MAX_WHEEL_SPEED)		v2=-MAX_WHEEL_SPEED;
	else ;
	
	if((v2>-5) && (v2<5))  v2=0;
	
	if(v3>MAX_WHEEL_SPEED) 		v3=MAX_WHEEL_SPEED;
	else if(v3< -MAX_WHEEL_SPEED)		v3=-MAX_WHEEL_SPEED;
	else ;
	
	if((v3>-5) && (v3<5))  v3=0;
	
	if(v4>MAX_WHEEL_SPEED) 		v4=MAX_WHEEL_SPEED;
	else if(v4< -MAX_WHEEL_SPEED)		v4=-MAX_WHEEL_SPEED;
	else ;	
	
	if((v4>-5) && (v4<5))  v4=0;
   
	set_spd[0] = v1;
	set_spd[1] = v2;
	set_spd[2] = v3;
	set_spd[3] = v4;
}

/**
* @funtion			计算PID输出，并通过CAN总线发送出去
* @Brief        供定时中断函数调用,每调用一次函数，则计算一次PID输出
*								并将计算结果通过函数 CAN_DJI_C620_DataSend(iq1,iq2,iq3,iq4) 下发到电机
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DJI_Motor_Control(void)
{
	unsigned char i=0;// 电机速度控制
	for(i=0; i<4; i++)
	{
		// set_spd 为设置的目标速度
		// pid_spd 为四个目标结构体
		// moto_chassis 为测量的转速
		pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
	}
	// 发送到四个电机的转速CAN总线上
	CAN_DJI_C620_DataSend(pid_spd[0].pos_out,pid_spd[1].pos_out,pid_spd[2].pos_out,pid_spd[3].pos_out);				 
}

/**
* @funtion			大疆C620 电调CAN总线数据发送函数
* @Brief        
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CanTxMsg TxMessage;
	TxMessage.StdId = 0x200;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = iq1 >> 8;
	TxMessage.Data[1] = iq1;
	TxMessage.Data[2] = iq2 >> 8;
	TxMessage.Data[3] = iq2;
	TxMessage.Data[4] = iq3 >> 8;
	TxMessage.Data[5] = iq3;
	TxMessage.Data[6] = iq4 >> 8;
	TxMessage.Data[7] = iq4;
	CAN_Transmit(CAN1, &TxMessage); // 发送消息  
}


///*
//*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
//*/
//void get_total_angle(moto_measure_t *p)
//{
//	int res1, res2, delta;
//	if(p->angle < p->last_angle)
//	{			//可能的情况
//		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
//		res2 = p->angle - p->last_angle;				//反转	delta=-
//	}
//	else
//	{	//angle > last
//		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
//		res2 = p->angle - p->last_angle;				//正转	delta +
//	}
//	//不管正反转，肯定是转的角度小的那个是真的
//	if(ABS(res1)<ABS(res2))
//		delta = res1;
//	else
//		delta = res2;
// 
//	p->total_angle += delta;
//	p->last_angle = p->angle;
//}
//电机上报数据频率为1KHZ
void CAN_RxCpltCallback(CanRxMsg* RxMessage)
{
	static uint8_t i;
	//ignore can1 or can2.
	switch(RxMessage->StdId)
		{
		case CAN_3510Moto1_ID:
		case CAN_3510Moto2_ID:
		case CAN_3510Moto3_ID:
		case CAN_3510Moto4_ID:
			{
				i = RxMessage->StdId - CAN_3510Moto1_ID;
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], RxMessage) : get_moto_measure(&moto_chassis[i], RxMessage);
				get_moto_measure(&moto_info,RxMessage);
			}
			break;
	   }
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    分析,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	
	
	int delta=0;

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ; //电机转角
	
	ptr->real_current  = (int16_t)(RxMessage->Data[2]<<8 | RxMessage->Data[3]);
	ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
	
	ptr->given_current = (int16_t)(RxMessage->Data[4]<<8 | RxMessage->Data[5])/-5;
	ptr->Temp = RxMessage->Data[6];
	
	if(ptr->speed_rpm > 10 ) //电机正转
	{
		if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - ptr->last_angle;
		}
		else if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta = ptr->angle + 8192 - ptr->last_angle;
			ptr->round_cnt++;
		}
		else;
	}
  else if(ptr->speed_rpm < -10) //电机反转
	{
		if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta =ptr->angle - ptr->last_angle;
		}
		else if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - 8192 - ptr->last_angle;
			ptr->round_cnt--;
		}
		else;
	}
	else
		delta=0;
	//ptr->total_angle += delta;
 
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

 
// 获取电机静止时候的值
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}



 /*
 * 函数名： USB_LP_CAN1_RX0_IRQHandler
 * 描述 ： USB 中断和 CAN 接收中断服务程序， USB 跟 CAN 公用 I/O，这里只用到 CAN 的中断。
 * 输入 ：无
 * 输出 : 无
 * 调用 ：无
 
 void USB_LP_CAN1_RX0_IRQHandler(void)
 {
 CanRxMsg RxMessage;

 RxMessage.StdId=0x00;
 RxMessage.ExtId=0x00;
 RxMessage.IDE=0;
 RxMessage.DLC=0;
 RxMessage.FMI=0;
 RxMessage.Data[0]=0x00;
 RxMessage.Data[1]=0x00;

 CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

 if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
 && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
 {
 ret = 1;
 }
else
{
ret = 0;
 }
}
*/


//------------------------------串口接收指令协议-------------------------------//
//将串口2的数据转存到数组 UART2_ReBuff中

volatile uint8_t UART2_ReBuff[100];  
volatile uint16_t UART2_ReCont;  
volatile unsigned char UART2_Reflag; 

void DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength)
{
	uint16_t i =0;
	uint16_t rpm_offset =10000; //转速偏移一万转
	uint16_t speed_offset =10; //速度偏移10m/s
	
	for(i=0;i<datalength;i++)
	{
			UART2_ReBuff[i] = *DataBuff;
			DataBuff++;
	}
 	 
	UART2_ReCont = datalength;
	UART2_Reflag =0x01;
	
 recived_cmd.cmd=UART2_ReBuff[3];
	
 if(recived_cmd.cmd ==0xF1)
 {
			recived_cmd.tag_rpm1 = UART2_ReBuff[4]*256+UART2_ReBuff[5] - rpm_offset;
			recived_cmd.tag_rpm2 = UART2_ReBuff[6]*256+UART2_ReBuff[7] - rpm_offset;
			recived_cmd.tag_rpm3 = UART2_ReBuff[8]*256+UART2_ReBuff[9] - rpm_offset;
			recived_cmd.tag_rpm4 = UART2_ReBuff[10]*256+UART2_ReBuff[11] - rpm_offset;
	    recived_cmd.rpm_available=0x01;
	    recived_cmd.speed_available=0x00;
 }
 else if(recived_cmd.cmd ==0xF2)
 {
			recived_cmd.tag_speed_x = (UART2_ReBuff[4]*256+UART2_ReBuff[5])/100.0-speed_offset;
			recived_cmd.tag_speed_y = (UART2_ReBuff[6]*256+UART2_ReBuff[7])/100.0-speed_offset;
			recived_cmd.tag_speed_z = (UART2_ReBuff[8]*256+UART2_ReBuff[9])/100.0-speed_offset;
	 
	    recived_cmd.rpm_available=0x00;
	    recived_cmd.speed_available=0x01;
 }
 else
 {
	    //recived_cmd.rpm_available=0x00;
	    //recived_cmd.speed_available=0x00;
	 ;
 }
 recived_cmd.flag=0x01;

 
	#if DEBUUG_Motor_RECIVED
		UART_send_string(USART2,"\n  Uart2 recived  data length:  ");
		UART_send_data(USART2,UART2_ReCont);
	  UART_send_string(USART2,"  Byte");
	#endif	
}
/**
*@funtion   串口打印调试信息
*@Brief 以文本的方式打印电机的速度信息 调试使用
*@data 20200816  
*/
void DJI_Motor_Show_Message(void)
{
	unsigned char i=0;
	if(recived_cmd.flag)
	{
		if(recived_cmd.cmd == 0xF1)
		{
				UART_send_string(USART2,"DJI_Motor:"); 
				UART_send_string(USART2,"tag_rpm1:");UART_send_data(USART2,recived_cmd.tag_rpm1);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_rpm2:");UART_send_data(USART2,recived_cmd.tag_rpm2);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_rpm3:");UART_send_data(USART2,recived_cmd.tag_rpm3);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_rpm4:");UART_send_data(USART2,recived_cmd.tag_rpm4);UART_send_char(USART2,'\n');	
		}
   	else if(recived_cmd.cmd == 0xF2)
		{
				UART_send_string(USART2,"DJI_Motor:"); 
				UART_send_string(USART2,"tag_speed_x:");UART_send_floatdat(USART2,recived_cmd.tag_speed_x);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_speed_y:");UART_send_floatdat(USART2,recived_cmd.tag_speed_y);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_speed_z:");UART_send_floatdat(USART2,recived_cmd.tag_speed_z);UART_send_char(USART2,'\n');	
		}
    else;
	
	}
	for(i=0;i<4;i++)
	{
		// 打印四个电机的转速、转角、温度等信息
		UART_send_string(USART2,"M"); UART_send_data(USART2,i);UART_send_string(USART2,"： ");
		UART_send_string(USART2,"v:");UART_send_floatdat(USART2,moto_chassis[i].speed_rpm);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"t_a:");UART_send_floatdat(USART2,moto_chassis[i].total_angle);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"n:");UART_send_floatdat(USART2,moto_chassis[i].round_cnt);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"a:");UART_send_floatdat(USART2,moto_chassis[i].angle);UART_send_char(USART2,'\n');	
	}
	UART_send_char(USART2,'\n');	
	UART_send_char(USART2,'\n');	
 
}

/**
*@funtion   电机上报信息到PC上
*@Brief 按照预定的协议格式上传四个电机的转速、位置、温度到工控机 
*@data 20200816  
*/
void DJI_Motor_Upload_Message(void)
{
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
	
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x01;
	
	  //上传数据帧计数
		senddat[i++]=(motor_upload_counter>>24);
		senddat[i++]=(motor_upload_counter>>16);
		senddat[i++]=(motor_upload_counter>>8);
		senddat[i++]=(motor_upload_counter);
	
		for(j=0;j<4;j++) //4*11个字节
		{
			senddat[i++] = moto_chassis[j].speed_rpm>>8; //int16
			senddat[i++] = moto_chassis[j].speed_rpm;

			senddat[i++] = moto_chassis[j].total_angle>>24;
			senddat[i++] = moto_chassis[j].total_angle>>16;	
			senddat[i++] = moto_chassis[j].total_angle>>8;		
			senddat[i++] = moto_chassis[j].total_angle;	

			senddat[i++] = moto_chassis[j].round_cnt>>24;
			senddat[i++] = moto_chassis[j].round_cnt>>16;	
			senddat[i++] = moto_chassis[j].round_cnt>>8; //int16
			senddat[i++] = moto_chassis[j].round_cnt;
			
			senddat[i++] = moto_chassis[j].angle>>8; //int16
			senddat[i++] = moto_chassis[j].angle;

			senddat[i++] = moto_chassis[j].Temp;
		}
		
		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		motor_upload_counter++;
}


/**
*@funtion   在ROS节点启动时候清除里程计，使之从零开始计数
*@Brief 当上位机按照固定指令下发时候，启动该函数 清楚里程计的累计值
*@data 20200816 
*/
void DJI_Motor_Clear_Odom(void)
{
	unsigned char j=0;
		for(j=0;j<4;j++)  
		{
			moto_chassis[j].msg_cnt=0;  

			moto_chassis[j].angle=0;
			moto_chassis[j].last_angle=0;
			moto_chassis[j].speed_rpm=0;  
			moto_chassis[j].real_current=0;  
			moto_chassis[j].given_current=0;  
						
			moto_chassis[j].Temp=0;
			moto_chassis[j].offset_angle=0;
			moto_chassis[j].round_cnt=0;
			moto_chassis[j].total_angle=0;	

		}
		
		motor_upload_counter=0;
		UART_send_string(USART2,"OK");
}



//------------------------------------------PID相关参数---------------------------//
/*
* 功能：PID初始化 
*
*20190916  CRP
*参考：大疆官网C620控制代码
*/
void PID_struct_init( pid_t *pid, uint32_t mode,uint32_t maxout,uint32_t intergral_limit,
    float 	kp,float 	ki,float 	kd)
{
    /*init function pointer*/
   // pid->f_param_init = pid_param_init;
   // pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
	
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
	
		if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
			return 0;
		if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
			return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
		
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid->i) >= 0.001f)
					pid->iout += pid->i * pid->err[NOW];
				else
					pid->iout = 0;
        pid->dout = -pid->d * gyro/100.0f;	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->IntegralLimit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

//中途更改参数设定
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
