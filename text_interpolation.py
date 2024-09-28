from pynput import keyboard
import time
from datetime import datetime
import math
import pickle
import time
import numpy as np
import json

from dora import Node


node = Node()

key_value = ""
key_press_flag = 0

# 每0.01秒刷新一次
try:
    linear_x =0.0
    linear_y =0.0
    linear_z =0.0  
    angular_x =0.0
    angular_y =0.0
    angular_z =0.0  
    speed_value_ratio = 0.1
    cnt = 0
    cnt_flag = 0
    #while True:
    for event in node:
        #event = node.next()
        event_type = event["type"]        
        if event_type == "INPUT":
            #if event["id"] == "keyboard":
            key_value = event["value"][0].as_py()
            print("recived: ",key_value, flush=True)
            #print(event["id"])
            cnt = cnt + 1
            linear_x = 0
            angular_z = 0

            if key_value == "quit":
                break
            elif key_value == "forward":
                linear_x = 1.0 * speed_value_ratio
                angular_z = 0.0
                key_press_flag = 0
                cnt_flag = 0
                #print("forward  linear_x",linear_x, flush=True)
            elif key_value == "left":
                linear_x = 0.0
                angular_z = 1 * speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
                #print("left", flush=True)
            elif key_value == "right":
                linear_x = 0.0
                angular_z = -1.0 * speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
                #print("right", flush=True)
            elif key_value == "backward":
                linear_x = -1.0 * speed_value_ratio
                angular_z = 0.0
                key_press_flag = 0
                cnt_flag = 0
                #print("back", flush=True)
            elif key_value == "stop":
                linear_x = 0.0
                angular_z = 0.0
                key_press_flag = 0
                cnt_flag = 0
                #print("stop", flush=True)

            #if key_press_flag == 0:
            #    cnt_flag = cnt_flag + 1
            #if cnt_flag > 5:
                # linear_x = 0
                # linear_y = 0
                # linear_z = 0
                # angular_x = 0
                # angular_y = 0
                # angular_z = 0

            timestamp = datetime.now().timestamp()  # 获取当前时间
            # timestamp_ms = round(datetime.now().timestamp() * 1000)# 转换为毫秒格式
            #print(timestamp)

            sentence_dict = {
                "header": {
                    "frame_id": "keyboard",
                    "seq": cnt,
                    "stamp": {
                        "sec": int(timestamp),
                        "nanosec": int((timestamp - int(timestamp)) * 1e9),
                    },
                },
                "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
                "angular": {"x": angular_x, "y": angular_y, "z": angular_z},
            }
            json_string = json.dumps(
                sentence_dict, indent=4
            )  # 使用indent参数设置缩进宽度为4
            print(json_string, flush=True)
            json_bytes = json_string.encode("utf-8")
            node.send_output(
                "CmdVelTwist",
                json_bytes,
                event["metadata"],
            )

            # print("counter: ",self.cnt1 ,"NavSatFix  pub time: ", timestamp_ms)# 打印时间戳（毫秒）

            key_value = None
            # time.sleep(0.02)
        elif event_type == "STOP":
            print("received stop")
            break
        else:
            print("received unexpected event:", event_type)
            break

except KeyboardInterrupt:
    print("Keyboard Interrupt")
    pass
