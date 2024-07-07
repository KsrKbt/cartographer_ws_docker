import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from cartographer_ros_msgs.msg import SubmapList
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import websocket
import json
import threading
import traceback

class CartographerSubscriberRepublisher(Node):

    def __init__(self):
        super().__init__('cartographer_subscriber_republisher')
        
        # QoSプロファイルの設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # TF用のQoSプロファイル
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=100
        )

        # TFメッセージの購読設定
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            qos_profile=tf_qos
        )

        # SubmapListメッセージの購読設定
        self.submap_list_subscription = self.create_subscription(
            SubmapList,
            '/submap_list',
            self.submap_list_callback,
            qos_profile
        )

        # WebSocket接続の設定 (IPアドレスを実際のDocker ホストのIPに変更してください)
        self.ws = websocket.WebSocketApp("ws://172.17.0.1:9090",
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)

        # WebSocketを別スレッドで実行
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.ws_thread.start()

        self.get_logger().info('Cartographer Subscriber Republisher has been initialized.')

    def tf_callback(self, msg):
        self.get_logger().info('Received TF message.')
        try:
            msg_dict = self.tf_to_dict(msg)
            self.get_logger().info(f'Converted TF message: {msg_dict}')
            self.publish_to_bridge('/tf', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in tf_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def submap_list_callback(self, msg):
        self.get_logger().info('Received SubmapList message.')
        try:
            msg_dict = self.submap_list_to_dict(msg)
            self.get_logger().info(f'Converted SubmapList message: {msg_dict}')
            self.publish_to_bridge('/submap_list', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in submap_list_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def publish_to_bridge(self, topic, msg_dict):
        data = {
            "op": "publish",
            "topic": topic,
            "msg": msg_dict
        }
        try:
            json_data = json.dumps(data)
            self.get_logger().info(f'Sending to rosbridge: {json_data}')
            self.ws.send(json_data)
            self.get_logger().info(f'Published {topic} message to rosbridge.')
        except Exception as e:
            self.get_logger().error(f'Error in publish_to_bridge: {e}')
            self.get_logger().error(traceback.format_exc())

    def tf_to_dict(self, msg):
        return {
            "transforms": [
                {
                    "header": {
                        "stamp": {
                            "sec": t.header.stamp.sec,
                            "nanosec": t.header.stamp.nanosec
                        },
                        "frame_id": t.header.frame_id
                    },
                    "child_frame_id": t.child_frame_id,
                    "transform": {
                        "translation": {
                            "x": t.transform.translation.x,
                            "y": t.transform.translation.y,
                            "z": t.transform.translation.z
                        },
                        "rotation": {
                            "x": t.transform.rotation.x,
                            "y": t.transform.rotation.y,
                            "z": t.transform.rotation.z,
                            "w": t.transform.rotation.w
                        }
                    }
                } for t in msg.transforms
            ]
        }

    def submap_list_to_dict(self, msg):
        return {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "submap": [
                {
                    "trajectory_id": s.trajectory_id,
                    "submap_index": s.submap_index,
                    "pose": {
                        "position": {
                            "x": s.pose.position.x,
                            "y": s.pose.position.y,
                            "z": s.pose.position.z
                        },
                        "orientation": {
                            "x": s.pose.orientation.x,
                            "y": s.pose.orientation.y,
                            "z": s.pose.orientation.z,
                            "w": s.pose.orientation.w
                        }
                    },
                    "is_frozen": s.is_frozen
                } for s in msg.submap
            ]
        }

    def on_open(self, ws):
        self.get_logger().info("Connected to rosbridge server")

    def on_message(self, ws, message):
        self.get_logger().info(f"Received message from rosbridge: {message}")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")
        self.get_logger().error(traceback.format_exc())

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info(f"Disconnected from rosbridge server. Status code: {close_status_code}, Message: {close_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = CartographerSubscriberRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ws.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
