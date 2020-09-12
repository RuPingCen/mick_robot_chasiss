# mick_robot_chasiss
代码默认的分支master分支适用于四轮差速小车，与MickX4分支相同

## MickM4分支
MickM4分支为麦克纳姆轮ROS底盘的底盘控制程序，代码适用于STM32F103及C620电调，更多的信息可以参考博客地址：https://blog.csdn.net/crp997576280/article/details/102026459

```
 git clone -b MickM4 https://github.com/RuPingCen/mick_robot_chasiss.git
```
![MickM4](https://github.com/RuPingCen/mick_robot_chasiss/raw/master/Reference/MickM4.png)

## MickX4分支
MickX4分支为四轮差速小车分支

```
 git clone -b MickX4 https://github.com/RuPingCen/mick_robot_chasiss.git
```

操作说明，左上角开关S1为使能选择，S1开关置于最上为不使能遥控器，S1开关拨到最下为使能遥控器；右上角开关S2为速度选择开关，上中下对应 1,2，3个档位，1档速度最大值为 1m/s，2档速度最大值为2m/s,3档最大速度为3.5m/s。
        
![MickX4](https://github.com/RuPingCen/mick_robot_chasiss/raw/master/Reference/mickx4.png)

通讯协议的接口说明请参考链接： 【腾讯文档】ROS底盘数据帧协议v1.1
https://docs.qq.com/sheet/DV2hmSEdSYVVtclB4

# 更新日志

2020-9-8
	1、更新了DBUS中的函数名称
	2、更新遥控器信号丢失造成的数据乱码引起“疯转”的问题
	3、统一4轮和2轮差速小车模型电机控制函数的单位为 m/s  和 rad/s 
 
 2019-10-07
	1、添加ROS节点下发命令清零里程计数据功能
 
 2020-9-9 
	1、更新MickM4 麦克纳姆轮底盘的代码
