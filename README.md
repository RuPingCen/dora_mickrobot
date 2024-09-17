# Overview

该节点主要基于 C/C++实现，节点读取键盘数据并控制 mickrobot 四轮差速小车移动

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

启动 dora 节点

```
dora start  mickrobot_dataflow.yml --name test
```

## 查看打印信息

```
 dora logs test mickrobot_chassis
```

## 节点说明

该节点接收 CmdVelTwist 的 json 字符串流，获取 json 字符中以下数据对小车底盘进行控制

```
j_cmd_vel["linear"]["x"];
j_cmd_vel["linear"]["y"];
j_cmd_vel["linear"]["z"];
j_cmd_vel["angular"]["x"];
j_cmd_vel["angular"]["y"];
j_cmd_vel["angular"]["z"];
```

同时，小车底盘会以 100Hz 的频率对外发布小车的状态信息（x 方向速度，y 方向速度，旋转角速度），发布的 Json 字符串数据流名称为“Odometry”

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

## Qwenvl2 recorder

### Keyboard teleoperation without camera

First try:

```bash
dora up
dora build keyboard_teleop_no_recording.yml
dora start keyboard_teleop_no_recording.yml
# 2 Ctrl-C for stopping
```

> This is going to start a keyboard teleoperation session without camera

### Keyboard operation with camera and recording

```bash
# Clone llama factory
cd ..
git clone --depth 1 https://github.com/hiyouga/LLaMA-Factory.git
cd ../dora_mickrobot
dora build qwenvl2_recorder.yml
dora start qwenvl2_recorder.yml
```

## Qwenvl2 fintuning

Within llama factory folder,

- Install all dependency of llama factory

- Modify `examples/train_lora/qwen2vl_lora_sft.yaml` so that the dataset is the one you want to use,

```yaml,diff
- dataset: mllm_demo,identity  # video: mllm_video_demo
+ dataset: dora_demo_107,dora_demo_108,identity`
```

- You can also choose the 2B model instead of the 7B model with

```yaml,diff
- model_name_or_path: Qwen/Qwen2-VL-7B-Instruct
+ model_name_or_path: Qwen/Qwen2-VL-2B-Instruct
```

- Then start finetuning with:

```bash
llamafactory-cli train examples/train_lora/qwen2vl_lora_sft.yaml
```

You can then

### Qwenvl2

```bash
# Clone llama factory
cd ..
git clone --depth 1 https://github.com/hiyouga/LLaMA-Factory.git
cd ../dora_mickrobot
dora build qwenvl2_recorder.yml
dora start qwenvl2_recorder.yml
```
