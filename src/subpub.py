import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from cartographer_ros_msgs.msg import SubmapList
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose, PoseStamped, Point, Twist
import websocket
import json
import threading

class CartographerDataTransferNode(Node):
    def __init__(self):
        super().__init__('cartographer_data_transfer_node')
        
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.submap_list_sub = self.create_subscription(SubmapList, '/submap_list', self.submap_list_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.ws = websocket.WebSocketApp("ws://192.168.1.23:9090",
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
        
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.ws_thread.start()

    def tf_callback(self, msg):
        self.send_to_pc1('/tf', msg)

    def submap_list_callback(self, msg):
        # サブマップリストは内部処理用なので転送しない
        pass

    def map_callback(self, msg):
        self.send_to_pc1('/map', msg)

    def send_to_pc1(self, topic, msg):
        data = {
            "op": "publish",
            "topic": topic,
            "msg": self.ros_msg_to_dict(msg)
        }
        self.ws.send(json.dumps(data))

    def on_message(self, ws, message):
        data = json.loads(message)
        if data['topic'] == '/scan':
            scan_msg = self.dict_to_scan(data['msg'])
            self.scan_pub.publish(scan_msg)
            self.get_logger().info('Received and republished scan data from PC1')
        elif data['topic'] == '/odom':
            odom_msg = self.dict_to_odom(data['msg'])
            self.odom_pub.publish(odom_msg)
            self.get_logger().info('Received and republished odom data from PC1')

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection opened")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info(f"WebSocket connection closed: {close_status_code} - {close_msg}")

    def ros_msg_to_dict(self, msg):
        if isinstance(msg, TFMessage):
            return {
                "transforms": [self.transform_stamped_to_dict(tf) for tf in msg.transforms]
            }
        elif isinstance(msg, OccupancyGrid):
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
        else:
            self.get_logger().warn(f"Unsupported message type: {type(msg)}")
            return {}

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

    def pose_to_dict(self, pose):
        return {
            "position": self.point_to_dict(pose.position),
            "orientation": self.quaternion_to_dict(pose.orientation)
        }

    def point_to_dict(self, point):
        return {"x": point.x, "y": point.y, "z": point.z}

    def quaternion_to_dict(self, quaternion):
        return {"x": quaternion.x, "y": quaternion.y, "z": quaternion.z, "w": quaternion.w}

    def vector3_to_dict(self, vector3):
        return {"x": vector3.x, "y": vector3.y, "z": vector3.z}

    def dict_to_scan(self, data):
        scan_msg = LaserScan()
        scan_msg.header = self.dict_to_header(data['header'])
        scan_msg.angle_min = data['angle_min']
        scan_msg.angle_max = data['angle_max']
        scan_msg.angle_increment = data['angle_increment']
        scan_msg.time_increment = data['time_increment']
        scan_msg.scan_time = data['scan_time']
        scan_msg.range_min = data['range_min']
        scan_msg.range_max = data['range_max']
        scan_msg.ranges = data['ranges']
        scan_msg.intensities = data['intensities']
        return scan_msg

    def dict_to_odom(self, data):
        odom_msg = Odometry()
        odom_msg.header = self.dict_to_header(data['header'])
        odom_msg.child_frame_id = data['child_frame_id']
        odom_msg.pose = self.dict_to_pose_with_covariance(data['pose'])
        odom_msg.twist = self.dict_to_twist_with_covariance(data['twist'])
        return odom_msg

    def dict_to_header(self, header_dict):
        header = Header()
        header.stamp.sec = header_dict['stamp']['sec']
        header.stamp.nanosec = header_dict['stamp']['nanosec']
        header.frame_id = header_dict['frame_id']
        return header

    def dict_to_pose_with_covariance(self, pose_with_cov_dict):
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = self.dict_to_pose(pose_with_cov_dict['pose'])
        pose_with_cov.covariance = pose_with_cov_dict['covariance']
        return pose_with_cov

    def dict_to_twist_with_covariance(self, twist_with_cov_dict):
        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist
