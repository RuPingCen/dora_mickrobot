from dora import Node
import pyarrow as pa

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "keyboard":
            char = event["value"][0].as_py()
            if char == "w":
                print(" w ",event["value"])
                node.send_output("text", pa.array(["forward"]))
            elif char == "s":
                node.send_output("text", pa.array(["backward"]))
                # node.send_output("text", pa.array(["stop"]))
            elif char == "d":
                node.send_output("text", pa.array(["right"]))
            elif char == "a":
                node.send_output("text", pa.array(["left"]))
            elif char =="q":
                node.send_output("text", pa.array(["stop"]))
