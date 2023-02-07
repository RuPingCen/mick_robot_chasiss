# mick_robot_chasiss

项目文档：https://mickrobot.github.io/

代码默认的分支master分支适用于四轮差速小车，X4分支为第一版小车代码分支。 M4分支为麦克纳姆轮第一版小车代码分支。小车的**PCB**、**代码**及**3D图纸**均是开源，大家可以自行下载打样学习。注意：新开源的电源板和控制板仅供大家自己打样学习用，切勿用于商业用途。有问题可联系 cenruping@vip.qq.com

欢迎加QQ群讨论  1149897304 (开源ROS自主导航小车)


**目录说明**

STM32_Code：存放小车上嵌入式控制板的代码文件

3D_Model_xxx：存放小车3D模型文件

PCB_File：存放小车上所用PCB文件

Reference_Documents：存放相关的传感器的使用手册等

## 1 更新日志
2022-4-20

	1、增加了对SBUS控制信号的解码程序，支持SBUS遥控器
	2、对代码增加了注释，

2021-4-18

	1、开源了小车的电源板和小车控制板
	2、将DBUS、上位机发送的命令移到中断函数中进行处理
	3、将PID相关计算函数移到PID.c中
	4、添加MPU9250 读取和姿态计算函数 

2020-9-8

	1、更新了DBUS中的函数名称
	2、更新遥控器信号丢失造成的数据乱码引起“疯转”的问题
	3、统一4轮和2轮差速小车模型电机控制函数的单位为 m/s  和 rad/s 

 2019-10-07

	1、添加ROS节点下发命令清零里程计数据功能

 2020-9-9 

	1、更新MickM4 麦克纳姆轮底盘的代码

## 2 MickX4分支
MickX4分支为四轮差速小车分支

```
 git clone -b MickX4 https://github.com/RuPingCen/mick_robot_chasiss.git
```

操作说明：以大疆遥控器为例，左上角开关S1为使能选择，S1开关置于最上为不使能遥控器（此时由上位机控制），S1开关拨到最下为使能遥控器；右上角开关S2为速度选择开关，上中下对应 1,2，3个档位，1档速度最大值为 1m/s，2档速度最大值为2m/s,3档最大速度为3.5m/s。

中文博客：https://rupingcen.blog.csdn.net/article/details/117257934

通讯协议的接口说明请参考链接： 【腾讯文档】ROS底盘数据帧协议v1.1
https://docs.qq.com/sheet/DV2hmSEdSYVVtclB4

![MickX4](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig//mickx4.png)

## 3 MickM4分支
MickM4分支为麦克纳姆轮ROS底盘的底盘控制程序，代码适用于STM32F103及C620电调

更多的信息可以参考博客地址：https://blog.csdn.net/crp997576280/article/details/102026459

```
 git clone -b MickM4 https://github.com/RuPingCen/mick_robot_chasiss.git
```
![MickM4](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/MickM4.png)

## 4 小车控制板

### 4.1 小车控制板接口

控制板输入20-36V DC直流，对外提供1路DC 5V 2A 、1路DC 12V 2A 对车载传感器供电。

![control_fig0](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/control_fig0.png)

提供1路DBUS接口、1路232接口、1路CAN总线、1路485接口、1路UART3接口（与485复用）、1路IIC。其中DBUS被用来接收遥航模遥控器的数据，232接口负责与上位机ROS通讯。CAN总线连接4个M3508电机。IIC连接板子上安装的MPU9250。485接口和UART3接口复用，可扩展其他传感器模块。
3路LED指示灯用于显示程序状态。2路按键、4路拨码开关用于调试和选择程序功能。4路隔离输入（输入电压范围12-24V）。4路隔离输出（输出高阻态和GND，承受电流2A）。

![control_fig1](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/control_fig1.png)

### 4.2 小车控制板外形尺寸

板子外形为99*99 mm 安装孔位于四周呈轴对称分布，孔中心间距为93mm,孔直径为φ3.1 mm。如图5所示。
![control_fig4](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/control_fig4.png)


## 5 小车电源板
如下图所示，电源板输入20-36V DC直流，输出1路DC 5V 2A 、1路DC 3.3-19V 2A 可调电源 、1路DC 12V 3.5A、1路DC 19V 3.5A，可满足对工控机和自主导航小车车载传感器供电需求。
![pwr_fig1](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/pwr_fig1.png)

### 5.1 电源板参数

![pwr_fig1](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/pwr_fig3.png)

经过电子负载实际测试，5V和3.3V-19V 采用LM2596S方案,每一路可实现2A的稳定输出，加装散热片以后可实现2.5A长时间输出。
12V 在电子负载实测中可以达到长时间稳定输出3.5A 输出，加装扇热片以后可以实现4A长时间输出，短时可达4.5A。
19V在电子负载实测中可以达到长时间稳定输出3.5A 输出，加装扇热片以后可以实现4A长时间输出，短时可达4.5A，如下图所示。
![pwr_fig2](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/pwr_fig2.png)

### 5.2 外形尺寸
板子外形为99*99 mm 安装孔位于四周呈轴对称分布，孔中心间距为93mm,孔直径为φ3.1 mm。如图6所示。
![pwr_fig6](https://raw.githubusercontent.com/RuPingCen/blog/master/mick_robot_chasiss/fig/pwr_fig6.png)
