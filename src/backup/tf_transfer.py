import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import websocket
import json
import threading
import time

class TFTransferNode(Node):
    def __init__(self):
        super().__init__('tf_transfer_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile)
        
        self.ws = None
        self.connect_websocket()

    def tf_callback(self, msg):
        self.send_to_pc1('/tf', msg)

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
            "transforms": [self.transform_stamped_to_dict(tf) for tf in msg.transforms]
        }

    def transform_stamped_to_dict(self, tf):
        return {
            "header": self.header_to_dict(tf.header),
            "child_frame_id": tf.child_frame_id,
            "transform": {
                "translation": self.vector3_to_dict(tf.transform.translation),
                "rotation": self.quaternion_to_dict(tf.transform.rotation)
            }
        }

    def header_to_dict(self, header):
        return {
            "stamp": {
                "sec": header.stamp.sec,
                "nanosec": header.stamp.nanosec
            },
            "frame_id": header.frame_id
        }

    def vector3_to_dict(self, vector3):
        return {"x": vector3.x, "y": vector3.y, "z": vector3.z}

    def quaternion_to_dict(self, quaternion):
        return {"x": quaternion.x, "y": quaternion.y, "z": quaternion.z, "w": quaternion.w}

def main():
    rclpy.init()
    node = TFTransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()