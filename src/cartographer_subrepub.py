import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from cartographer_ros_msgs.msg import SubmapList
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterEvent, Log
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

        # サブスクリプションの設定
        self.tf_subscription = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, qos_profile=tf_qos)
        self.submap_list_subscription = self.create_subscription(
            SubmapList, '/submap_list', self.submap_list_callback, qos_profile)
        self.constraint_list_subscription = self.create_subscription(
            MarkerArray, '/constraint_list', self.constraint_list_callback, qos_profile)
        self.landmark_poses_list_subscription = self.create_subscription(
            MarkerArray, '/landmark_poses_list', self.landmark_poses_list_callback, qos_profile)
        self.parameter_events_subscription = self.create_subscription(
            ParameterEvent, '/parameter_events', self.parameter_events_callback, qos_profile)
        self.rosout_subscription = self.create_subscription(
            Log, '/rosout', self.rosout_callback, qos_profile)
        self.scan_matched_points2_subscription = self.create_subscription(
            PointCloud2, '/scan_matched_points2', self.scan_matched_points2_callback, qos_profile)
        self.trajectory_node_list_subscription = self.create_subscription(
            MarkerArray, '/trajectory_node_list', self.trajectory_node_list_callback, qos_profile)

        self.ws = websocket.WebSocketApp("ws://192.168.64.58:9090",
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
            self.publish_to_bridge('/tf', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in tf_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def submap_list_callback(self, msg):
        self.get_logger().info('Received SubmapList message.')
        try:
            msg_dict = self.submap_list_to_dict(msg)
            self.publish_to_bridge('/submap_list', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in submap_list_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def constraint_list_callback(self, msg):
        self.get_logger().info('Received ConstraintList message.')
        try:
            msg_dict = self.marker_array_to_dict(msg)
            self.publish_to_bridge('/constraint_list', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in constraint_list_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def landmark_poses_list_callback(self, msg):
        self.get_logger().info('Received LandmarkPosesList message.')
        try:
            msg_dict = self.marker_array_to_dict(msg)
            self.publish_to_bridge('/landmark_poses_list', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in landmark_poses_list_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def parameter_events_callback(self, msg):
        self.get_logger().info('Received ParameterEvent message.')
        try:
            msg_dict = self.parameter_event_to_dict(msg)
            self.publish_to_bridge('/parameter_events', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in parameter_events_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def rosout_callback(self, msg):
        self.get_logger().info('Received Rosout message.')
        try:
            msg_dict = self.log_to_dict(msg)
            self.publish_to_bridge('/rosout', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in rosout_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def scan_matched_points2_callback(self, msg):
        self.get_logger().info('Received ScanMatchedPoints2 message.')
        try:
            msg_dict = self.point_cloud2_to_dict(msg)
            self.publish_to_bridge('/scan_matched_points2', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in scan_matched_points2_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def trajectory_node_list_callback(self, msg):
        self.get_logger().info('Received TrajectoryNodeList message.')
        try:
            msg_dict = self.marker_array_to_dict(msg)
            self.publish_to_bridge('/trajectory_node_list', msg_dict)
        except Exception as e:
            self.get_logger().error(f'Error in trajectory_node_list_callback: {e}')
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

    def marker_array_to_dict(self, msg):
        return {
            "markers": [
                {
                    "header": {
                        "stamp": {
                            "sec": m.header.stamp.sec,
                            "nanosec": m.header.stamp.nanosec
                        },
                        "frame_id": m.header.frame_id
                    },
                    "ns": m.ns,
                    "id": m.id,
                    "type": m.type,
                    "action": m.action,
                    "pose": {
                        "position": {
                            "x": m.pose.position.x,
                            "y": m.pose.position.y,
                            "z": m.pose.position.z
                        },
                        "orientation": {
                            "x": m.pose.orientation.x,
                            "y": m.pose.orientation.y,
                            "z": m.pose.orientation.z,
                            "w": m.pose.orientation.w
                        }
                    },
                    "scale": {
                        "x": m.scale.x,
                        "y": m.scale.y,
                        "z": m.scale.z
                    },
                    "color": {
                        "r": m.color.r,
                        "g": m.color.g,
                        "b": m.color.b,
                        "a": m.color.a
                    },
                    "lifetime": {
                        "sec": m.lifetime.sec,
                        "nanosec": m.lifetime.nanosec
                    },
                    "frame_locked": m.frame_locked,
                    "points": [{"x": p.x, "y": p.y, "z": p.z} for p in m.points],
                    "colors": [{"r": c.r, "g": c.g, "b": c.b, "a": c.a} for c in m.colors],
                    "text": m.text,
                    "mesh_resource": m.mesh_resource,
                    "mesh_use_embedded_materials": m.mesh_use_embedded_materials
                } for m in msg.markers
            ]
        }

    def parameter_event_to_dict(self, msg):
        return {
            "stamp": {
                "sec": msg.stamp.sec,
                "nanosec": msg.stamp.nanosec
            },
            "node": msg.node,
            "new_parameters": [
                {
                    "name": p.name,
                    "value": self.parameter_value_to_dict(p.value)
                } for p in msg.new_parameters
            ],
            "changed_parameters": [
                {
                    "name": p.name,
                    "value": self.parameter_value_to_dict(p.value)
                } for p in msg.changed_parameters
            ],
            "deleted_parameters": [
                {
                    "name": p.name
                } for p in msg.deleted_parameters
            ]
        }

    def parameter_value_to_dict(self, value):
        if value.type == ParameterType.PARAMETER_BOOL:
            return {"type": "bool", "value": value.bool_value}
        elif value.type == ParameterType.PARAMETER_INTEGER:
            return {"type": "integer", "value": value.integer_value}
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            return {"type": "double", "value": value.double_value}
        elif value.type == ParameterType.PARAMETER_STRING:
            return {"type": "string", "value": value.string_value}
        elif value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return {"type": "byte_array", "value": list(value.byte_array_value)}
        elif value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return {"type": "bool_array", "value": list(value.bool_array_value)}
        elif value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return {"type": "integer_array", "value": list(value.integer_array_value)}
        elif value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return {"type": "double_array", "value": list(value.double_array_value)}
        elif value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return {"type": "string_array", "value": list(value.string_array_value)}
        else:
            return {"type": "unknown", "value": None}

    def log_to_dict(self, msg):
        return {
            "stamp": {
                "sec": msg.stamp.sec,
                "nanosec": msg.stamp.nanosec
            },
            "level": msg.level,
            "name": msg.name,
            "msg": msg.msg,
            "file": msg.file,
            "function": msg.function,
            "line": msg.line
        }

    def point_cloud2_to_dict(self, msg):
        return {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "height": msg.height,
            "width": msg.width,
            "fields": [
                {
                    "name": f.name,
                    "offset": f.offset,
                    "datatype": f.datatype,
                    "count": f.count
                } for f in msg.fields
            ],
            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "data": list(msg.data),  # バイナリデータをリストに変換
            "is_dense": msg.is_dense
        }
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