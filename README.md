# Overview

该节点主要基于C/C++实现，节点读取键盘数据并控制 mickrobot 四轮差速小车移动

1. **w a d x** ：表示前后左右

2. **上 下 左 右**（方向键） ：表示前后左右

3. **s** 表示停止

4. 数字键 **1 2 3** 表示速度 1m/s 2m/s 3m/s

注：使用该节点时候需要打开左上角遥控器开关，设置到自动驾驶模式。 

## 启动命令：

为串口添加权限

```
sudo chmod 777 /dev/ttyUSB0
```
启动dora节点
```
dora start  mickrobot_dataflow.yml --name test
```
##  查看打印信息 

```
 dora logs test mickrobot_chassis
```

## 节点说明

该节点接收 CmdVelTwist 的json字符串流，获取json字符中以下数据对小车底盘进行控制

```
j_cmd_vel["linear"]["x"];
j_cmd_vel["linear"]["y"];
j_cmd_vel["linear"]["z"];
j_cmd_vel["angular"]["x"];
j_cmd_vel["angular"]["y"];
j_cmd_vel["angular"]["z"];
```

同时，小车底盘会以100Hz的频率对外发布小车的状态信息（x方向速度，y方向速度，旋转角速度），发布的Json字符串数据流名称为“Odometry”

```
	# chassis position
	j_odom_pub["pose"]["position"]["x"] = position_x;
	j_odom_pub["pose"]["position"]["y"] = position_y;
	j_odom_pub["pose"]["position"]["z"] = 0;
	j_odom_pub["pose"]["orientation"]["x"] = 0;
	j_odom_pub["pose"]["orientation"]["y"] = 0;
	j_odom_pub["pose"]["orientation"]["z"] = 0;
	j_odom_pub["pose"]["orientation"]["w"] = 1;
	# chassis speed
	j_odom_pub["twist"]["linear"]["x"] = linear_x;
	j_odom_pub["twist"]["linear"]["y"] = linear_y;
	j_odom_pub["twist"]["linear"]["z"] = 0;
	j_odom_pub["twist"]["angular"]["x"] = 0;
	j_odom_pub["twist"]["angular"]["y"] = 0;
	j_odom_pub["twist"]["angular"]["z"] = linear_w;
```

