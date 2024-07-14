import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from builtin_interfaces.msg import Time
import websocket
import json
import threading

class TFRepublisherNode(Node):
    def __init__(self):
        super().__init__('tf_republisher_node')
        
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        
        self.ws = websocket.WebSocketApp("ws://192.168.1.23:9090",
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
        
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.ws_thread.start()

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection opened")

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data['topic'] == '/tf':
                tf_msg = TFMessage()
                for transform in data['msg']['transforms']:
                    tf_stamped = self.dict_to_transform_stamped(transform)
                    tf_msg.transforms.append(tf_stamped)
                self.tf_pub.publish(tf_msg)
                self.get_logger().debug(f"Republished TF message with {len(tf_msg.transforms)} transforms")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON message")
        except KeyError as e:
            self.get_logger().error(f"Missing key in message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def dict_to_transform_stamped(self, transform_dict):
        tf_stamped = TransformStamped()
        
        # Header
        tf_stamped.header.stamp = Time(
            sec=transform_dict['header']['stamp']['sec'],
            nanosec=transform_dict['header']['stamp']['nanosec']
        )
        tf_stamped.header.frame_id = transform_dict['header']['frame_id']
        
        # Child frame ID
        tf_stamped.child_frame_id = transform_dict['child_frame_id']
        
        # Translation
        tf_stamped.transform.translation = Vector3(
            x=transform_dict['transform']['translation']['x'],
            y=transform_dict['transform']['translation']['y'],
            z=transform_dict['transform']['translation']['z']
        )
        
        # Rotation
        tf_stamped.transform.rotation = Quaternion(
            x=transform_dict['transform']['rotation']['x'],
            y=transform_dict['transform']['rotation']['y'],
            z=transform_dict['transform']['rotation']['z'],
            w=transform_dict['transform']['rotation']['w']
        )
        
        return tf_stamped

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info(f"WebSocket connection closed: {close_status_code} - {close_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()