# mick_robot_bringup

该代码为差速、4WS4WD全向、麦克纳姆轮ROS底盘的ROS节点，与mick_robot_chasiss工程代码配合使用

项目主页：https://github.com/RuPingCen/mick_robot_chasiss

底盘硬件框架信息可以参考博客地址：https://blog.csdn.net/crp997576280/article/details/102026459

# 下载安装
 1.安装依赖项

    sudo apt-get install ros-melodic-serial

 2. cd catkin_ws/src

 3. (需要下载安装包并手动解压) git clone https://github.com/RuPingCen/mick_robot_bringup.git

 4. catkin_make

 5. 针对差速底盘使用带X4标号的启动文件  roslaunch mick_bringup mickx4_bringup.launch

 6. 针对麦克纳姆轮底盘使用带M4标号的启动文件  roslaunch mick_bringup mickm4_bringup.launch

# 参数说明
dev（串口设备名称）

baud（串口波特率,默认值115200）

hz（里程计发布频率）

chassis_type（底盘类型选择） **0**： 表示差速底盘 **1**：表示 麦克纳姆轮底盘  **2**：4WS4WD移动底盘  **3**： 阿卡曼转向底盘

sub_cmdvel_topic(接收话题名称)

pub_odom_topic（里程计发布话题名称）

is_pub_path（选择是否发布里程计）

### V1.2 修改日志
  １．利用6050DMP输出的陀螺仪和yaw角修改差速底盘里程计计算模型
### V1.1 修改日志
  １．修改了ROS serial库参数，使得串口读取数据帧更加稳定

  ２．增加节点启动时候清零里程计的指令

  ３．修复了里程计掉头以后由于方向原因使得位置估算错误的BUG

  ４．增加了参数传递功能，可通过launch文件传递参数

### Ｖ1.0 修改日志
  第一次提交


