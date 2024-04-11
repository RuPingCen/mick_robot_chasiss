/**
 * mick四轮差速底盘， 
 * 1、接收cmd_vel 话题的数据，将其转化成转速指令 
 * 2、然后下发到底盘的STM32控制器中
 *  注意：该四轮差速模型与两轮差速模型相同，发送数据的时候需要把1/2号电机的速度设置为一样
 * 3/4号电机速度设置为一样，
  * 
 * 增加IMU数据上传
 * 增加超声波上传
 * 
 * maker:crp
 * 2017-5-13
 */

#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 
#include <serial/serial.h>
#include <std_msgs/String.h>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <chassis_mick_msg.h>
using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

int chassis_type = 0; //默认采用差速模式  0：差速  1-麦克纳姆轮  2: Ackermann  3:4WS4WD
int is_pub_path = 0; //默认不发布小车底盘轨迹  0：不发布   1 发布

float WHEEL_RATIO =19.0; 		//减速比 3508电机减速比为1:19
float WHEEL_K=0.355;            // 麦克纳母轮模式

float WHEEL_L=0.680;                 //左右轮子的间距
float WHEEL_D=0.1525;//0.254; 		    	//轮子直径  10寸的轮子
float WHEEL_R=WHEEL_D/2.0; 			//轮子半径
float WHEEL_PI=3.141693; 			//pi

 
 
nav_msgs::Path path;
serial::Serial ros_ser;
ros::Publisher odom_pub,path_pub;

 
volatile rc_info_t rc;
int rc_init_flags =0;
unsigned int init_times = 0;
int sum_offset[4] = {0};
int show_message =1;
float RC_MIN = 0, RC_MAX = 2500, RC_K = 1; //遥控器摇杆通道输出的最小值、最大值、比例系数
ros::Publisher joy_pub;

moto_measure_t moto_chassis[4] = {0};
moto_measure_t moto_rmd_chassis[4] = {0};
imu_measure_t imu_chassis;  //IMU 数据
//uint16_t Ultrasonic_data [10];   //超声波数据
vector<uint16_t> Ultrasonic_data(10,0);

union floatData //union的作用为实现char数组和float之间的转换
{
    int32_t int32_dat;
    unsigned char byte_data[4];
}motor_upload_counter,total_angle,round_cnt;
union IntData //union的作用为实现char数组和int16数据类型之间的转换
{
    int16_t int16_dat;
    unsigned char byte_data[2];
}speed_rpm,imu;

 
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
void send_speed_to_X4chassis(float x,float y,float w);
void send_rpm_to_chassis( int w1, int w2, int w3, int w4);
void send_speed_to_4WS4WDchassis(float x,float y,float w );
void send_speed_to_Ackerchassis(float x,float w );

void send_rpm_to_4WS4WDchassis(vector<float> vw);

void clear_odometry_chassis(void);
bool analy_uart_recive_data( std_msgs::String serial_data);
void calculate_position_for_odometry(void);
void calculate_chassisAckermann_position_for_odometry(void);
void calculate_chassisAckermann2_position_for_odometry(void);
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w);
bool publish_joy_msg(void);
int calculate_rc_offset(void);


int main(int argc,char** argv)
{
    string out_result;
    bool uart_recive_flag;
 
//     unsigned char buf[200];                      //定义字符串长度
//     boost::asio::io_service iosev;
//     serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
//     sp.set_option(serial_port::baud_rate(115200));
//     sp.set_option(serial_port::flow_control());
//     sp.set_option(serial_port::parity());
//     sp.set_option(serial_port::stop_bits());
//     sp.set_option(serial_port::character_size(8));
 
    string sub_cmdvel_topic,pub_odom_topic,dev,joy_topic;
	int baud,time_out,hz;
 	ros::init(argc, argv, "mick robot");
	ros::NodeHandle n("~");
	 
	n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic, "/cmd_vel");
	n.param<std::string>("pub_odom_topic", pub_odom_topic, "/odom");
	n.param<std::string>("dev", dev, "/dev/mick");
	n.param<int>("baud", baud, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 100);
	n.param<int>("is_pub_path", is_pub_path, 0); // 默认不发布小车底盘轨迹
	n.param<int>("chassis_type", chassis_type, 3); //  

	n.param<std::string>("joy_topic", joy_topic, "/rc_remotes/joy");
	n.param<float>("RC_K", RC_K, 1);
	n.param<float>("RC_MIN", RC_MIN, 0);
	n.param<float>("RC_MAX", RC_MAX, 2500);

	
	ROS_INFO_STREAM("sub_cmdvel_topic:   "<<sub_cmdvel_topic);
	ROS_INFO_STREAM("pub_odom_topic:   "<<pub_odom_topic);
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("baud:   "<<baud);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	ROS_INFO_STREAM("is_pub_path:   "<<chassis_type<<"\t  0:No  1:Yes"); 
	ROS_INFO_STREAM("chassis_type:   "<<chassis_type<<"\t  0:X4  1:M4  2: Ackermann  3:4WS4WD"); 

	ROS_INFO_STREAM("RC_K:   " << RC_K);
	ROS_INFO_STREAM("RC_MIN:   " << RC_MIN);
	ROS_INFO_STREAM("RC_MAX:   " << RC_MAX);
	
	//订阅主题command
	ros::Subscriber command_sub = n.subscribe(sub_cmdvel_topic, 10, cmd_vel_callback);
	//发布主题sensor
	// ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
	
	joy_pub = n.advertise<sensor_msgs::Joy>(joy_topic, 20);
	odom_pub= n.advertise<nav_msgs::Odometry>(pub_odom_topic, 20); //定义要发布/odom主题
	path_pub = n.advertise<nav_msgs::Path>(pub_odom_topic+"/path",20, true);
	// 开启串口模块
	 try
	 {
		ros_ser.setPort(dev);
		ros_ser.setBaudrate(baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		//serial::Timeout to = serial::Timeout(1,time_out,0,time_out,0);
		to.inter_byte_timeout=1;
		to.read_timeout_constant=5;
		to.read_timeout_multiplier=0;
		ros_ser.setTimeout(to);
		ros_ser.open();
		ros_ser.flushInput(); //清空缓冲区数据
	 }
	 catch (serial::IOException& e)
	 {
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if(ros_ser.isOpen())
	{
		ros_ser.flushInput(); //清空缓冲区数据
		ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
	    return -1;
	}
 
	ros::Rate loop_rate(hz);

	clear_odometry_chassis();
	bool init_OK=false;
	int init_odom_cnt =0;
	while((!init_OK) && (init_odom_cnt<10))	
	{
		clear_odometry_chassis();
		init_odom_cnt++;
		ROS_INFO_STREAM("clear odometry ..... ");
		if(ros_ser.available())
		{
			std_msgs::String serial_data;
			string str_tem;
			//获取串口数据
			serial_data.data = ros_ser.read(ros_ser.available());
			str_tem =  serial_data.data;
			// cout<<"Recived "<<serial_data.data.c_str()<<endl;
			// ROS_INFO_STREAM(serial_data.data.c_str());
			if(str_tem.find("OK",0) )
			{
				init_OK =true;
				ros_ser.flushInput(); //清空缓冲区数据
			}
		}
		sleep(1);
	}
	ROS_INFO_STREAM("clear odometry successful !");
   
    while(ros::ok())
    { 
		if(ros_ser.available() )
		{
			//ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String serial_data;
			//获取串口数据
			serial_data.data = ros_ser.read(ros_ser.available());
			//cout<<serial_data.data << "\n"<<endl;
			uart_recive_flag = analy_uart_recive_data(serial_data);
			if(uart_recive_flag)
			{
				uart_recive_flag=0;
				if(chassis_type == 0 || chassis_type == 1)
				{
					calculate_position_for_odometry();
				}
				else if(chassis_type == 2)
				{
					 calculate_chassisAckermann_position_for_odometry();
				}
				else
				{
					;
				}
			    // odom_pub.publish(serial_data);//将串口数据发布到主题sensor
			}
			else
			{
				ROS_WARN_STREAM(" analy uart recive data error ...");
				//serial_data.data = ros_ser.read(ros_ser.available());
				ros_ser.flushInput(); //清空缓冲区数据
				sleep(0.2);            //延时0.1秒,确保有数据进入
			}
		}
		ros::spinOnce();
		loop_rate.sleep();	
    }
   
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();
    return 1;
}
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
//ROS_INFO_STREAM("Write to serial port" << msg->data);
 // ostringstream os;
  float speed_x,speed_y,speed_w;
  float v1=0,v2=0,v3=0,v4=0;
  // os<<"speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z<<'\n';
 //cout<<os.str()<<endl;
//send_speed_to_chassis(msg->linear.x*10,msg->linear.y*10,msg->angular.z*2);

  
  speed_x = msg->linear.x;
  speed_y = 0;
  speed_w = msg->angular.z;
  

	if(chassis_type == 0) //差速模式
	{
		v1 =speed_x-0.5*WHEEL_L*speed_w;   //左边    //转化为每个轮子的线速度
		v2 =v1;
		v4 =-(speed_x+0.5*WHEEL_L*speed_w);
		v3 =v4;
	}
	else if(chassis_type == 1) // 麦克纳姆轮模式
	{
		v1 =speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
		v2 =speed_x+speed_y-WHEEL_K*speed_w;
		v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
		v4 =-(speed_x+speed_y+WHEEL_K*speed_w);
	}
	else if(chassis_type == 2) // 阿卡曼模式
	{
		send_speed_to_Ackerchassis(speed_x, speed_w); //直接发送目标速度 和 角速度
		//ROS_INFO_STREAM("send_speed_to_Ackerchassis: "<<speed_x<<"  "<<speed_w);
		return ;
	}
	else if(chassis_type == 3) // 4WS4WD模式
	{
			send_speed_to_4WS4WDchassis(speed_x,speed_y,speed_w);
			ROS_INFO_STREAM("send_speed_to_4WS4WDchassis: "<<speed_x<<"  "<<speed_y<<"  "<<speed_w);
		 	return ;
			// speed_x =-1;
			// speed_y= 1;
			// float v = sqrt(speed_x*speed_x+speed_y*speed_y);


			// float theta = acosf(speed_x/v)*57.3;
			// if(theta>90) //量化到[-pi pi]  
			// 	theta=180-theta;
			// // if(speed_y<0) //位于3/4象限
			// // {
			// // 	theta = -theta;
			// // }
			// if(speed_x<0)  
			// {
			// 	v = -v;
			// }
			// float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1515926*0.258); 

			// if(isnanf(theta))
			// {
			// 	ROS_WARN_STREAM("theta is nan ");
			// 	theta=0;
			// }
			// if(theta>92) 
			// 	theta=92;
			// else  if(theta<-92) 
			// 	theta=-92;
			// else;

			// if(RPM>300) 
			// 	RPM=300;
 

			// vector<float> vw;
			// vw.push_back(theta);	vw.push_back(-RPM);   // theta  °/s
			// vw.push_back(theta);	vw.push_back(-RPM);  // RPM  转 /min
			// vw.push_back(theta);	vw.push_back(RPM); 
			// vw.push_back(theta);	vw.push_back(RPM); 
			// vw.push_back(0);	vw.push_back(0);
			// vw.push_back(0);	vw.push_back(0);
			// send_rpm_to_4WS4WDchassis(vw);
			// ROS_INFO_STREAM("send_rpm_to_4WS4WDchassis   theta  : "<<theta<<" RPM "<<RPM);
			// return ;
	}
	else
	{
		ROS_WARN_STREAM("unknown chassis type ! ");
	}

	v1 =v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度　RPM
	v2 =v2/(2.0*WHEEL_R*WHEEL_PI);
	v3 =v3/(2.0*WHEEL_R*WHEEL_PI);
	v4 =v4/(2.0*WHEEL_R*WHEEL_PI);

	v1 =v1*WHEEL_RATIO*60;    //转每秒转换到RPM
	v2 =v2*WHEEL_RATIO*60;
	v3 =v3*WHEEL_RATIO*60;
	v4 =v4*WHEEL_RATIO*60;

  send_rpm_to_chassis(v1,v2,v3,v4);	 
 //send_rpm_to_chassis(200,200,200,200);	
  ROS_INFO_STREAM("v1: "<<v1<<"      v2: "<<v2<<"      v3: "<<v3<<"      v4: "<<v4);
  ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z);
}

// 差速小车
void send_speed_to_X4chassis(float x,float y,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
  cmd =0xF3; //针对MickX4的小车使用F3 字段      针对MickM4的小车使用F2
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((x+speed_0ffset)*100);
  data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
  data_tem[counter++] =((y+speed_0ffset)*100);
  data_tem[counter++] =((w+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((w+speed_0ffset)*100);
  data_tem[counter++] =0x00;
  data_tem[counter++] =0x00;
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
  ros_ser.write(data_tem,counter);
}
 
/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　RPM
 */
void send_rpm_to_chassis( int w1, int w2, int w3, int w4)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10000; //转速偏移10000转

  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
 cmd =0xF1;
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
 
  data_tem[counter++] =(w1+speed_0ffset)/256; // 
  data_tem[counter++] =(w1+speed_0ffset)%256;
  
  data_tem[counter++] =(w2+speed_0ffset)/256; // 
  data_tem[counter++] =(w2+speed_0ffset)%256;
  
  data_tem[counter++] =(w3+speed_0ffset)/256; // 
  data_tem[counter++] =(w3+speed_0ffset)%256;
  
  data_tem[counter++] =(w4+speed_0ffset)/256; // 
  data_tem[counter++] =(w4+speed_0ffset)%256;
 
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
 
 ros_ser.write(data_tem,counter);
}

void send_speed_to_Ackerchassis(float x,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
  cmd =0xF4;  
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((x+speed_0ffset)*100);
  data_tem[counter++] =0; // Y
  data_tem[counter++] =0;
  data_tem[counter++] =((w+speed_0ffset)*100)/256; //  w
  data_tem[counter++] =((w+speed_0ffset)*100);

  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
  ros_ser.write(data_tem,counter);
}
/**
 * @function  发送X Y 方向的速度 以及旋转角速度
 * ＠param  X Y 单位 m/s    w  单位　rad/s
 */
void send_speed_to_4WS4WDchassis(float x,float y,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd=0xF2; 
  unsigned int check=0;
  
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((x+speed_0ffset)*100);
  data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
  data_tem[counter++] =((y+speed_0ffset)*100);
  data_tem[counter++] =((w+speed_0ffset)*100)/256; // W
  data_tem[counter++] =((w+speed_0ffset)*100);
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
  ros_ser.write(data_tem,counter);
}

/**********************************************************
 * @function  发送6个电机的转速到底盘控制器
 * ＠param vw 表示 (w1 v1 w2 v2 .... w6 v6)
 * 电机的转速 单位　RPM
 * 角度    单位  °度
**********************************************************/
void send_rpm_to_4WS4WDchassis(vector<float> vw)
{
	if(vw.size()<12)
	{	
		ROS_WARN_STREAM("For 4ws4wd chasiss, the length of vm < 12");
		return;
	}

  uint8_t data_tem[50];
  unsigned int speed_0ffset=10000; //转速偏移10000转
   unsigned int theta_0ffset=360; 
  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
  cmd =0xFA;
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
 
	i=0;
	for(int j=0;j<6;j++)
	{
		data_tem[counter++] =((vw[ i]+theta_0ffset)*100)/256;
		data_tem[counter++] =((vw[i++]+theta_0ffset)*100);
		data_tem[counter++] =(vw[ i]+speed_0ffset)/256; // 
		data_tem[counter++] =(vw[ i++]+speed_0ffset);
	}
 
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
 
 ros_ser.write(data_tem,counter);
}

void clear_odometry_chassis(void)
{
  uint8_t data_tem[50];
  unsigned char i,counter=0;
  unsigned char  cmd,resave=0x00;
  unsigned int check=0;
  cmd =0xE1;
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  
  data_tem[counter++] =0x01; //  清零里程计
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
 
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
  data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
 
 ros_ser.write(data_tem,counter);
  
}
 
/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool  analy_uart_recive_data( std_msgs::String serial_data)
{
  unsigned char reviced_tem[500];
  uint16_t len=0,i=0,j=0;
  unsigned char check=0;
  unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
  uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
  len=serial_data.data.size();
  if(len<1 || len>500)
  {
	ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
	std_msgs::String serial_data;
	string str_tem;

	serial_data.data = ros_ser.read(ros_ser.available());
	str_tem =  serial_data.data;
     return false; //数据长度太短　
   }
   //ROS_INFO_STREAM("Read: " << serial_data.data.size() );

   // 有可能帧头不在第一个数组位置
  for( i=0;i<len;i++) 
  {
	 tem_last=  tem_curr;
	 tem_curr = serial_data.data.at(i);
	 if(tem_last == 0xAE && tem_curr==0xEA&&rec_flag==0) //在接受的数据串中找到帧头　
	 {
		 rec_flag=1;
		 reviced_tem[j++]=tem_last;
		 reviced_tem[j++]=tem_curr;
		 //ROS_INFO_STREAM("found frame head" ); 
	}
	else if (rec_flag==1)
	{
		reviced_tem[j++]=serial_data.data.at(i);
		if(tem_last == 0xEF && tem_curr==0xFE)
		{
			header_count++;
			rec_flag=0;
		}
	}
	else
		rec_flag=0;
  }
  // 检验数据长度和校验码是否正确
//   if(reviced_tem[len-3] ==check || reviced_tem[len-3]==0xff)
//     ;
//   else
//     return;
  // 检验接受数据的长度
  step=0;
  for(int k=0;k<header_count;k++) 
  {
	  len = (reviced_tem[2+step] +4 ) ; //第一个帧头的长度
	  //cout<<"read head :" <<i<< "      len:   "<<len;
	   if(reviced_tem[0+step] ==0xAE && reviced_tem[1+step] == 0xEA && reviced_tem[len-2+step]==0xEF &&reviced_tem[len-1+step]==0xFE) 
      {//检查帧头帧尾是否完整
		  if (reviced_tem[3+step] ==0x01 )
		  {
			  ROS_INFO_STREAM("recived motor  data" ); 
				i=4+step;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
				for(int j=0;j<4;j++)
				{
					speed_rpm.int16_dat=0;
					total_angle.int32_dat =0;
					round_cnt.int32_dat=0;
					
					speed_rpm.byte_data[1] = reviced_tem[i++] ; 
					speed_rpm.byte_data[0] = reviced_tem[i++] ;

					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1]=reviced_tem[i++];
					total_angle.byte_data[0]=reviced_tem[i++];
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;

					moto_chassis[j].angle = reviced_tem[i++] *256; 
					moto_chassis[j].angle += reviced_tem[i++];

					moto_chassis[j].Temp = reviced_tem[i++]; 

					moto_chassis[j].round_cnt =  round_cnt.int32_dat;
					moto_chassis[j].speed_rpm = speed_rpm.int16_dat;
					moto_chassis[j].total_angle = total_angle.int32_dat;
					
					moto_chassis[j].counter = motor_upload_counter.int32_dat;
				}
				// 根据电机安装的位置，第３号和第４号电机方向相反
				moto_chassis[2].speed_rpm = -moto_chassis[2].speed_rpm ;
				moto_chassis[2].total_angle = -moto_chassis[2].total_angle;
				moto_chassis[2].round_cnt = -moto_chassis[2].round_cnt;

				moto_chassis[3].speed_rpm = -moto_chassis[3].speed_rpm ;
				moto_chassis[3].total_angle = -moto_chassis[3].total_angle;
				moto_chassis[3].round_cnt = -moto_chassis[3].round_cnt;
			   //ROS_INFO_STREAM("recived motor data" ); 
			   	//for(j=0;j<4;j++)
				//{
					// 打印四个电机的转速、转角、温度等信息
					//ROS_INFO_STREAM("M "<< j <<": \t cnt: "<<moto_chassis[j].counter<<"\t V: "<<moto_chassis[j].speed_rpm<<"  t_a: "<<moto_chassis[j].total_angle <<"  n: "<<moto_chassis[j].round_cnt <<"  a: "<<moto_chassis[j].angle );
				   // ROS_INFO_STREAM("M "<< j <<": " <<motor_upload_counter.int32_dat ); 
					//ROS_INFO_STREAM("ｖ : "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle ); 
					//cout<<"M "<<j <<": " <<motor_upload_counter.int32_dat<<endl;
					//cout<<"ｖ: "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle<<endl;
				//}
		  }
		  else if (reviced_tem[3+step] ==0x05 ) //4WS4WD  8个电机状态数据
		  {
				i=4+step;
				for(int j=0;j<4;j++)
				{
					speed_rpm.int16_dat=0;
					total_angle.int32_dat =0;
					round_cnt.int32_dat=0;
					
					speed_rpm.byte_data[1] = reviced_tem[i++] ; 
					speed_rpm.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].speed_rpm = speed_rpm.int16_dat;			//*1000	
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].round_cnt =  round_cnt.int32_dat;
					
					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1] = reviced_tem[i++] ; 
					total_angle.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].angle =  total_angle.int32_dat;
 

					moto_rmd_chassis[j].angle = reviced_tem[i++] *256; // 转向电机角度
					moto_rmd_chassis[j].angle = reviced_tem[i++]; 
				}
				
				ROS_INFO_STREAM("recived 4WS4WD chassis motor data" ); 
				for(j=0;j<4;j++)
				{
					// 打印四个电机的转速、转角、温度等信息
					ROS_INFO_STREAM("M "<< j <<"\t V: "<<moto_chassis[j].speed_rpm
											<<": \t round_cnt: "<<moto_chassis[j].round_cnt
											<<"  angle: "<<moto_chassis[j].angle <<"  angle_rmd: "<<moto_rmd_chassis[j].angle );
				}
		  }
		  else if (reviced_tem[3+step] ==0x10 )
			{
			     i=4+step;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.ax = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.ay = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.az = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gx = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gy = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gz = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.mx = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.my = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.mz = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.pitch = imu.int16_dat/100.0f;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.roll = imu.int16_dat/100.0f;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.yaw = imu.int16_dat/100.0f;
			  
				ROS_INFO_STREAM("recived imu  data" ); 
			}
			else if (reviced_tem[3+step] ==0x11 )
			{
			    i=4+step;
				uint16_t ultra_tem=0;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
				for(int j =0;j<10;j++)
				{
				 ultra_tem=reviced_tem[i++]*256; 		ultra_tem = ultra_tem+reviced_tem[i++];
				 Ultrasonic_data.push_back(ultra_tem);
				}
				ROS_INFO_STREAM("recived Ulrat  data" ); 
			}
			else if (reviced_tem[3 + step] == 0xA3)
			{
				i = 4 + step;

				rc.ch1 = reviced_tem[i++];
				rc.ch1 = (rc.ch1 << 8) + reviced_tem[i++];
				rc.ch2 = reviced_tem[i++];
				rc.ch2 = (rc.ch2 << 8) + reviced_tem[i++];
				rc.ch3 = reviced_tem[i++];
				rc.ch3 = (rc.ch3 << 8) + reviced_tem[i++];
				rc.ch4 = reviced_tem[i++];
				rc.ch4 = (rc.ch4 << 8) + reviced_tem[i++];
				rc.sw1 = reviced_tem[i++];
				rc.sw2 = reviced_tem[i++];
				rc.sw3 = reviced_tem[i++];
				rc.sw4 = reviced_tem[i++];
				rc.type = reviced_tem[i++];// 1 DJI-DBUS   2 SBUS 遥控器类型
				rc.status = reviced_tem[i++];
				rc.update = 0x01;
				if (rc.ch1 >= (RC_MIN-200) && rc.ch1 <=(RC_MAX+200))
				{
					rc.available = 0x01;
				}
				else
				{
					ROS_WARN_STREAM("rc.chx < RC_MIN || rc.chx > RC_MAX");
				}
				// if(show_message)
				// {
				// 	ROS_INFO_STREAM("RC_Remotes date  ch1: " << rc.ch1 << " ch2: " << rc.ch2
				// 					 << " ch3: " << rc.ch3 << " ch4: " << rc.ch4 << " sw1: " 
				// 					 << rc.sw1 << " sw2: " << rc.sw2<< " sw3: " 
				// 					 << rc.sw3 << " sw4: " << rc.sw4<< " type: " << rc.type);
				// }
				return true;
			}
			else
			{
				ROS_WARN_STREAM("unrecognize frame" ); 
			}
			//return  true;
	  }
	   else
      {
		ROS_WARN_STREAM("frame head is wrong" ); 
         return  false;	
      }
      step+=len; 
  }
 return  true;	         
}
/**
 * @function 利用里程计数据实现位置估计
 * 
 */
float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
static int motor_init_flag = 0;
void calculate_position_for_odometry(void)
 {
  //方法１：　　计算每个轮子转动的位移，然后利用Ｆ矩阵合成Ｘ,Y,W三个方向的位移
  float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;
  float v1=0,v2=0,v3=0,v4=0;
  float K4_1 = 1.0/(4.0*WHEEL_K);
  float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
  float linear_x,linear_y,linear_w;
	 
  if(motor_init_flag == 0 && ((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0)))
  {
		s1 = (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s2 = (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s3 = (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s4 = (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		
		s1_last=s1;
		s2_last=s2;
		s3_last=s3;
		s4_last=s4;

		motor_init_flag++;//保证程序只进入一次
		
		return ;
  }
  
  s1_last=s1;
  s2_last=s2;
  s3_last=s3;
  s4_last=s4;
 
  //轮子转动的圈数乘以　N*２*pi*r
  s1 =    (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s2 =    (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s3 =    (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s4 =    (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
 
  s1_delta=s1-s1_last; //每个轮子位移的增量
  s2_delta=s2-s2_last;
  s3_delta=s3-s3_last;
  s4_delta=s4-s4_last;
  
   // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
   if(abs(s1_delta) < 0.001 )  s1_delta=0;
   if(abs(s2_delta) < 0.001 )  s2_delta=0;
    if(abs(s3_delta) < 0.001 )  s3_delta=0;
   if(abs(s4_delta) < 0.001 )  s4_delta=0;
   
  s1_delta = 0.5*s1_delta+0.5*s2_delta;  
  s4_delta = 0.5*s3_delta+0.5*s4_delta; 

//    if(s1_delta || s2_delta || s3_delta || s4_delta)
//   cout<<"s1_delta:  "<<s1_delta<<"   s2_delta: " <<s2_delta<<"   s3_delta: " <<s3_delta<<"   s4_delta: " <<s4_delta<<endl;
 
 if(chassis_type == 0) //差速模式
 {
	position_w_delta =((s4_delta)- (s1_delta))/float(WHEEL_L); //w 单位为弧度
	if(abs(position_w_delta) < 0.0001)
		position_r_delta=0;
	else
		position_r_delta = ((s4_delta)+(s1_delta))/float(2*position_w_delta);
	position_x_delta=position_r_delta*sin(position_w_delta);
	position_y_delta = position_r_delta*(1-cos(position_w_delta));
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------------

	position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
	position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
	position_w=position_w+position_w_delta;
 }
 else if(chassis_type == 1) // 麦克纳姆轮模式
 {
	position_x_delta= 0.25*s1_delta+ 0.25*s2_delta+ 0.25*s3_delta+ 0.25*s4_delta;
	position_y_delta = -0.25*s1_delta+ 0.25*s2_delta- 0.25*s3_delta+ 0.25*s4_delta;
	position_w_delta = -K4_1*s1_delta-K4_1*s2_delta+K4_1*s3_delta+ K4_1*s4_delta; //w 单位为弧度

	position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
	position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
	position_w=position_w+position_w_delta;
 }
 else if(chassis_type == 2) // 4WS4WD模式
 {
	  ROS_WARN_STREAM("4WS4WD  ! ");
 }
 else
 {
	 ROS_WARN_STREAM("unknown chassis type ! ");
 }
  


  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;

  v1 =    (moto_chassis[0].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2;
  v2 =    (moto_chassis[1].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
  v3 =    (moto_chassis[2].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
  v4 =    (moto_chassis[3].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
 if(chassis_type == 0) //差速模式
 {
  linear_x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
  linear_y = 0;
  linear_w = ((0.5*v3+0.5*v4)-(0.5*v1+0.5*v2))/float(WHEEL_L);
 }
 else if(chassis_type == 1) // 麦克纳姆轮模式
 {
  linear_x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
  linear_y = -0.25*v1+ 0.25*v2- 0.25*v3+ 0.25*v4;
  linear_w = -K4_1*v1-K4_1*v2+K4_1*v3+ K4_1*v4;
 }
  else if(chassis_type == 2) // 4WS4WD模式
 {
	  ROS_WARN_STREAM("4WS4WD  ! ");
 }
 else
 {
	 ROS_WARN_STREAM("unknown chassis type ! ");
 }
  
  ROS_INFO_STREAM("px:  "<<position_x<<"   py: " <<position_y<<"   pw: " <<position_w*57.3
  <<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w<<endl);
 
    publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
    //方法２;利用轮子的转速来推算
}


double last_time=0, curr_time =0;

// Ackerman底盘  速度计算
// 仅仅只是前轮转向模式
void calculate_chassisAckermann_position_for_odometry(void)
{
  float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
  float linear_x,linear_y,linear_w;
	 
  if(motor_init_flag == 0 && ((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0)))
  {
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = ros::Time::now().toSec();
		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
  }
    // float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1515926*0.258);
	float v_l = (moto_chassis[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
	float v_r = (moto_chassis[2].speed_rpm/1000.0)/70.02556;
 
 	// 利用轮子的转速来推算
	linear_x =( v_r + v_l)/2.0;
	linear_y = 0;
	linear_w =( v_r-v_l)/0.796; // 左右侧轮距 0.796             前后轮距 0.8083

	curr_time = ros::Time::now().toSec();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

	ROS_INFO_STREAM("  dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);

   
	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;

  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;

 	ROS_INFO_STREAM("  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
//针对前后转向的 阿卡曼模型
void calculate_chassisAckermann2_position_for_odometry(void)
{
	float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
	float linear_x,linear_y,linear_w;
		
	if(motor_init_flag == 0 && ((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0)))
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = ros::Time::now().toSec();
		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}
	// float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1515926*0.258);
	float v_1 = (moto_chassis[0].speed_rpm/1000.0)/70.02556;  
	float v_2 = (moto_chassis[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
	float v_3 = (moto_chassis[2].speed_rpm/1000.0)/70.02556;
	float v_4 = (moto_chassis[3].speed_rpm/1000.0)/70.02556;

	float theta_1 = (moto_rmd_chassis[0].angle)*0.0001745; //(0.01° -> rad) 
	float theta_2 = (moto_rmd_chassis[1].angle)*0.0001745;
	float theta_3 = (moto_rmd_chassis[2].angle)*0.0001745;
	float theta_4 = (moto_rmd_chassis[3].angle)*0.0001745;

		// 左右侧轮距 0.796    前后轮距 0.8083
	float rx = 0.796/2.0;
	float ry = 0.8083/2.0;
	float r2 = rx*rx + ry*ry;

	float m1 = rx*sin(theta_1)-ry*sin(theta_1)/(4*r2);
	float m2 = -rx*sin(theta_2)-ry*sin(theta_2)/(4*r2);
	float m3 = -rx*sin(theta_3)+ry*sin(theta_3)/(4*r2);
	float m4 = rx*sin(theta_4)+ry*sin(theta_4)/(4*r2);

	linear_x = (cos(theta_1)*v_1 + cos(theta_2)*v_2 + cos(theta_3)*v_3 + cos(theta_4)*v_4)/4.0;
	linear_y = (sin(theta_1)*v_1 + sin(theta_2)*v_2 + sin(theta_3)*v_3 + sin(theta_4)*v_4)/4.0;
	linear_w =  m1*v_1 + m2*v_2 + m3*v_3 + m4*v_4;

	// 利用轮子的转速来推算
	curr_time = ros::Time::now().toSec();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

	ROS_INFO_STREAM("  dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);


	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;

	if(position_w>2*WHEEL_PI)
	{
		position_w=position_w-2*WHEEL_PI;	
	}
	else if(position_w<-2*WHEEL_PI)
	{
		position_w=position_w+2*WHEEL_PI;
	}
	else;

 	ROS_INFO_STREAM("  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
/**
 * @function 发布里程计的数据
 * 
 */
void publish_odomtery(float  position_x,float position_y,float oriention,
						float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
	geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
	geometry_msgs::Quaternion odom_quat;   //四元数变量
	nav_msgs::Odometry odom;  //定义里程计对象
		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf::createQuaternionMsgFromYaw(oriention);//将偏航角转换成四元数

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = ros::Time::now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = position_x;
	odom_trans.transform.translation.y = position_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;        
	//发布tf坐标变化
	odom_broadcaster.sendTransform(odom_trans);

	//载入里程计时间戳
	odom.header.stamp = ros::Time::now(); 
	//里程计的父子坐标系
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";       
	//里程计位置数据：x,y,z,方向
	odom.pose.pose.position.x = position_x;     
	odom.pose.pose.position.y = position_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = vel_linear_x;
	odom.twist.twist.linear.y = vel_linear_y;
	odom.twist.twist.angular.z = vel_linear_w;    
	//发布里程计
	odom_pub.publish(odom);

	if(is_pub_path)
	{
		//发布小车里程计数据推算的轨迹
		geometry_msgs::PoseStamped this_pose_stamped;
		this_pose_stamped.pose = odom.pose.pose;

		path.header.stamp=odom.header.stamp;
		path.header.frame_id="odom";
		path.poses.push_back(this_pose_stamped);
		path_pub.publish(path);
	}
}
// 发布遥控器数据
bool publish_joy_msg(void)
{
	if(rc.update != 0x01)
	{
		ROS_WARN_STREAM("rc.update != 0x01 ");
		return false;
	}

	// 0 未标定   1 标定中   2标定完成
	if(rc_init_flags != 2)
	{
		ROS_WARN_STREAM("rc.state != 0x02");
		int flag = calculate_rc_offset();
		//ROS_WARN_STREAM("flag "<<flag<<"  rc.state "<<rc.state);
		if(flag == 0)
		{
			rc_init_flags = 0 ;
			ROS_WARN_STREAM("calculate_rc_offset failed .... ");
		}
		else if(flag == 1)
		{
			rc_init_flags =1 ;
			ROS_WARN_STREAM("initial .... ");
		}
	}
	else
	{
		if (rc.available == 1) 
		{
			 
			float ch1,ch2,ch3,ch4;
			 
			float rc_k = 1;

			if (rc.sw2 == 1)            rc_k = 1;
			else if (rc.sw2 == 3)   rc_k = 2;
			else if (rc.sw2 == 2)  	rc_k = 3; // 3m/s
			else 				rc_k = 0;
			
			// 设置死区
			ch1 = (rc.ch1 - rc.ch1_offset) / (RC_MAX - rc.ch1_offset);
			if(abs(ch1)<0.2) ch1=0;
			ch2 =(rc.ch2 - rc.ch2_offset) / (RC_MAX - rc.ch2_offset);
			if(abs(ch2)<0.2) ch2=0;
			ch3 = (rc.ch3 - rc.ch3_offset) / (RC_MAX - rc.ch3_offset);
			if(abs(ch3)<0.2) ch3=0;
			ch4 =(rc.ch4 - rc.ch4_offset) / (RC_MAX - rc.ch4_offset);
			if(abs(ch4)<0.2) ch4=0;

			sensor_msgs::Joy joy_msg;
			joy_msg.axes.push_back(RC_K *rc_k*ch1);
			joy_msg.axes.push_back(RC_K *rc_k*ch2);
			joy_msg.axes.push_back(RC_K *rc_k*ch3);
			joy_msg.axes.push_back(RC_K *rc_k*ch4);
			joy_msg.buttons.push_back(rc.sw1);
			joy_msg.buttons.push_back(rc.sw2);
			joy_pub.publish(joy_msg);
 
			return 1;
		}
	}
	rc.update = 0;
	 
	return 1;
}
// 计算遥控器的中位值
int calculate_rc_offset(void)
{
	int re_flag = 0;
	if(init_times < 20)
	{
		if ((rc.ch1 > 900 && rc.ch1 < 1100) && (rc.ch2 > 900 && rc.ch2 < 1100)
			&& (rc.ch3 > 900 && rc.ch3 < 1100) && (rc.ch4 > 900 && rc.ch4 < 1100))
		{
			sum_offset[0] += rc.ch1;
			sum_offset[1] += rc.ch2;
			sum_offset[2] += rc.ch3;
			sum_offset[3] += rc.ch4;
			rc.available = 0;
			rc_init_flags = 1; // 0 未标定   1 标定中   2标定完成
			init_times++;
			ROS_WARN_STREAM("calibrate...");
			re_flag = 1;
		}
	}
	else
	{
		rc.ch1_offset = sum_offset[0] / init_times;
		rc.ch2_offset = sum_offset[1] / init_times;
		rc.ch3_offset = sum_offset[2] / init_times;
		rc.ch4_offset = sum_offset[3] / init_times;
		ROS_INFO_STREAM("ch1_offset: " <<rc. ch1_offset << " ch2_offset: " << rc.ch2_offset
				 << "ch3_offset: " <<rc. ch3_offset << " ch4_offset: " << rc.ch4_offset);
		if (rc.ch1_offset == 0 || rc.ch2_offset == 0 || rc.ch3_offset == 0 || rc.ch4_offset == 0)
		{
			rc.available = 0;
			rc_init_flags = 0;
			init_times = 0;
			sum_offset[0] = 0;
			sum_offset[1] = 0;
			sum_offset[2] = 0;
			sum_offset[3] = 0;
			ROS_WARN_STREAM("calibrate faild...");
			re_flag = 0;
		}
		else
		{
			rc.available = 1;
			rc_init_flags = 0x02;
			ROS_INFO_STREAM("remote calibrate successful");
			re_flag = 2; //标定成功
		}
	}
	return re_flag; //标定中
}