import asyncio
import json
import time
import websocket
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("drone_3d_nav")

HOME_POINT = {"x": -12.0, "y": 0.0, "z": 1.0}

def send_3d_goal(x: float, y: float, z: float):
    """
    通过 WebSocket 向 /move_base_simple/goal 发送 nav_msgs/Path 目标点
    （Path 中包含一个 PoseStamped）
    """
    ws = websocket.create_connection("ws://127.0.0.1:9090")

    advertise_msg = {
        "op": "advertise",
        "topic": "/move_base_simple/goal",
        "type": "nav_msgs/Path"
    }
    ws.send(json.dumps(advertise_msg))
    time.sleep(0.3)

    goal_msg = {
        "op": "publish",
        "topic": "/move_base_simple/goal",
        "msg": {
            "header": {
                "frame_id": "map",
                "stamp": {"secs": 0, "nsecs": 0}
            },
            "poses": [
                {
                    "header": {
                        "frame_id": "map",
                        "stamp": {"secs": 0, "nsecs": 0}
                    },
                    "pose": {
                        "position": {"x": x, "y": y, "z": z},
                        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                    }
                }
            ]
        }
    }
    ws.send(json.dumps(goal_msg))
    time.sleep(0.1)

    ws.send(json.dumps({"op": "unadvertise", "topic": "/move_base_simple/goal"}))
    ws.close()

    return f"已发送三维目标点：X={x}, Y={y}, Z={z} (Path 格式)"

def get_current_pose(timeout=2.0):
    """
    通过 rosbridge 订阅 /visual_slam/odom (nav_msgs/Odometry)
    获取当前无人机位置 (x, y, z)
    """
    ws = websocket.create_connection("ws://127.0.0.1:9090", timeout=timeout)

    subscribe_msg = {
        "op": "subscribe",
        "topic": "/visual_slam/odom",
        "type": "nav_msgs/Odometry"
    }
    ws.send(json.dumps(subscribe_msg))

    ws.settimeout(timeout)
    try:
        raw = ws.recv()
        msg = json.loads(raw)
        if "msg" not in msg:
            raise ValueError("未收到有效消息")
        odom = msg["msg"]
        if "pose" in odom and "pose" in odom["pose"]:
            pos = odom["pose"]["pose"]["position"]
            x = pos["x"]
            y = pos["y"]
            z = pos["z"]
        else:
            raise ValueError("Odometry 消息缺少位置字段")
    except Exception as e:
        raise RuntimeError(f"获取当前位置失败: {e}")
    finally:
        ws.send(json.dumps({"op": "unsubscribe", "topic": "/visual_slam/odom"}))
        ws.close()

    return x, y, z

@mcp.tool()
def go_to_3d_goal(x: float, y: float, z: float):
    """控制无人机/机器人飞往三维目标点（x,y,z 单位：米）"""
    return send_3d_goal(x, y, z)

@mcp.tool()
def return_home():
    """返回当前设定的三维原点（HOME_POINT）"""
    send_3d_goal(HOME_POINT["x"], HOME_POINT["y"], HOME_POINT["z"])
    return f"已返回原点 ({HOME_POINT['x']}, {HOME_POINT['y']}, {HOME_POINT['z']})"

@mcp.tool()
def set_current_as_home():
    """将无人机当前所在位置（来自 /visual_slam/odom）设置为新的原点"""
    try:
        x, y, z = get_current_pose()
        HOME_POINT["x"] = x
        HOME_POINT["y"] = y
        HOME_POINT["z"] = z
        return f"已将当前位置设为原点：({x}, {y}, {z})"
    except Exception as e:
        return f"设置原点失败：{e}"

if __name__ == "__main__":
    asyncio.run(mcp.run(transport="stdio"))