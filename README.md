# Overview

This is a Dora program that uses the qwenvl2 model to realize the movement of the car. We want to use a camera installed on the car as the input of the large language model, and have the large language model output four instructions: forward, backward, turn left, and turn right to achieve autonomous driving of the car.

## mickrobot_chassis

The mickrobot_chassis node is implemented based on C/C++. This node gets keyboard value and controls the movement of the chassis

1. **w a d x**: indicates front, back, left, and right

2. **up, down, left, and right** (direction keys): indicates front, back, left, and right

3. **s** indicates stop

4. Number keys **1、2、 3** indicate speed 1**m/s 、2m/s、 3m/s**

==Note:== When using this node, you need to turn on the remote control switch in the upper left corner, which means setting the chassis to automatic driving mode.（使用该节点时候需要打开左上角遥控器开关，设置到自动驾驶模式）。

###   mickrobot_chassis node usage ：

step1：you should check dora  path in  “dora_mickrobot/CMakeLists.txt" line 47、48、60  and line 61 for mickrobot_chassis node path.



step2：unzip and build  thridpart_lib serial

```
cd dora_mickrobot\thridpart_lib
unzip  serial.zip
cd serial 
mkdir build
cmake ..
make
```

step3： build chassis node 

```
cd dora_mickrobot
mkdir build
cmake ..
make
```

step4： Grant permissions to the serial port

```
sudo chmod 777 /dev/ttyUSB0
```

step5： start mickrobot_chassis node with  Dora 

```
dora start  mickrobot_dataflow.yml --name test
```

### show chassis logs

```
 dora logs test mickrobot_chassis
```

### Chassis receiving/publishing message 

This mickrobot_chassis node receives the json string stream from **CmdVelTwist** and obtains the following data in the json string to control the chassis of the car

```
j_cmd_vel["linear"]["x"];
j_cmd_vel["linear"]["y"];
j_cmd_vel["linear"]["z"];
j_cmd_vel["angular"]["x"];
j_cmd_vel["angular"]["y"];
j_cmd_vel["angular"]["z"];
```

At the same time, the mickrobot_chassis node will publish the chassis status (x speed, y speed, rotational angular velocity) at a frequency of 100Hz. The name of the published Json string data stream is "Odometry"

```
# publish Odometry Json string
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
dora build qwenvl2.yml
dora start qwenvl2.yml
```



# Note:

1. For different camera devices, you need to modify the parameters of the camera node in the xxx.yml file
1.  The parameter LLAMA_FACTORY_ROOT_PATH in the yml file should give the absolute path
