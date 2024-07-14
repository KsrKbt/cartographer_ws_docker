import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import websocket
import json
import threading
import time

class MapTransferNode(Node):
    def __init__(self):
        super().__init__('map_transfer_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        
        self.ws = None
        self.connect_websocket()

    def map_callback(self, msg):
        self.send_to_pc1('/map', msg)

    def connect_websocket(self):
        while rclpy.ok():
            try:
                self.ws = websocket.WebSocketApp("ws://192.168.1.23:9090",
                                                 on_open=self.on_open,
                                                 on_message=self.on_message,
                                                 on_error=self.on_error,
                                                 on_close=self.on_close)
                self.ws_thread = threading.Thread(target=self.ws.run_forever)
                self.ws_thread.start()
                break
            except Exception as e:
                self.get_logger().error(f"Failed to connect to WebSocket: {e}")
                time.sleep(5)

    def send_to_pc1(self, topic, msg):
        data = {
            "op": "publish",
            "topic": topic,
            "msg": self.ros_msg_to_dict(msg)
        }
        try:
            if self.ws and self.ws.sock and self.ws.sock.connected:
                self.ws.send(json.dumps(data))
                self.get_logger().debug(f"Sent data to PC1: {topic}")
            else:
                self.get_logger().warn("WebSocket is not connected. Attempting to reconnect...")
                self.connect_websocket()
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

    def on_message(self, ws, message):
        self.get_logger().debug(f"Received message: {message}")

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection opened")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info(f"WebSocket connection closed: {close_status_code} - {close_msg}")
        self.connect_websocket()

    def ros_msg_to_dict(self, msg):
        return {
            "header": self.header_to_dict(msg.header),
            "info": {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": self.pose_to_dict(msg.info.origin)
            },
            "data": list(msg.data)
        }

    def header_to_dict(self, header):
        return {
            "stamp": {
                "sec": header.stamp.sec,
                "nanosec": header.stamp.nanosec
            },
            "frame_id": header.frame_id
        }

    def pose_to_dict(self, pose):
        return {
            "position": self.point_to_dict(pose.position),
            "orientation": self.quaternion_to_dict(pose.orientation)
        }

    def point_to_dict(self, point):
        return {"x": point.x, "y": point.y, "z": point.z}

    def quaternion_to_dict(self, quaternion):
        return {"x": quaternion.x, "y": quaternion.y, "z": quaternion.z, "w": quaternion.w}

def main():
    rclpy.init()
    node = MapTransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()