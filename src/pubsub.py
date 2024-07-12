import websocket
import json
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from rcl_interfaces.msg import ParameterEvent, Parameter, ParameterValue, ParameterType

class ScanOdomRepublisher(Node):
    def __init__(self):
        super().__init__('scan_odom_republisher')
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.parameter_events_publisher = self.create_publisher(ParameterEvent, '/parameter_events', 10)
        self.get_logger().info('Republisher node initialized')

    def publish_scan(self, scan_data):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = float(scan_data['angle_min'])
        msg.angle_max = float(scan_data['angle_max'])
        msg.angle_increment = float(scan_data['angle_increment'])
        msg.time_increment = float(scan_data['time_increment'])
        msg.scan_time = float(scan_data['scan_time'])
        msg.range_min = float(scan_data['range_min'])
        msg.range_max = float(scan_data['range_max'])
        msg.ranges = [float(r) if r is not None else float('inf') for r in scan_data['ranges']]
        msg.intensities = [float(i) if i is not None else 0.0 for i in scan_data['intensities']]
        self.scan_publisher.publish(msg)
        self.get_logger().info('Published LaserScan message')

    def publish_odom(self, odom_data):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Convert pose data
        pose = odom_data['pose']['pose']
        msg.pose.pose = Pose(
            position=Point(x=float(pose['position']['x']),
                           y=float(pose['position']['y']),
                           z=float(pose['position']['z'])),
            orientation=Quaternion(x=float(pose['orientation']['x']),
                                   y=float(pose['orientation']['y']),
                                   z=float(pose['orientation']['z']),
                                   w=float(pose['orientation']['w']))
        )

        # Convert twist data
        twist = odom_data['twist']['twist']
        msg.twist.twist = Twist(
            linear=Vector3(x=float(twist['linear']['x']),
                           y=float(twist['linear']['y']),
                           z=float(twist['linear']['z'])),
            angular=Vector3(x=float(twist['angular']['x']),
                            y=float(twist['angular']['y']),
                            z=float(twist['angular']['z']))
        )

        self.odom_publisher.publish(msg)
        self.get_logger().info('Published Odometry message')

    def publish_parameter_event(self, event_data):
        msg = ParameterEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.node = event_data['node']
        
        for new_param in event_data.get('new_parameters', []):
            param = Parameter()
            param.name = new_param['name']
            param.value = self.convert_parameter_value(new_param['value'])
            msg.new_parameters.append(param)
        
        for changed_param in event_data.get('changed_parameters', []):
            param = Parameter()
            param.name = changed_param['name']
            param.value = self.convert_parameter_value(changed_param['value'])
            msg.changed_parameters.append(param)
        
        msg.deleted_parameters = [Parameter(name=p['name']) for p in event_data.get('deleted_parameters', [])]
        
        self.parameter_events_publisher.publish(msg)
        self.get_logger().info('Published ParameterEvent message')

    def convert_parameter_value(self, value):
        param_value = ParameterValue()
        if isinstance(value, bool):
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = value
        elif isinstance(value, int):
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = value
        elif isinstance(value, float):
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = value
        elif isinstance(value, str):
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = value
        # Add more type conversions as needed
        return param_value

def on_message(ws, message, node):
    try:
        data = json.loads(message)
        node.get_logger().debug(f"Received data: {data}")
        if 'msg' in data:
            if data['topic'] == '/scan':
                node.publish_scan(data['msg'])
            elif data['topic'] == '/odom':
                node.publish_odom(data['msg'])
            elif data['topic'] == '/parameter_events':
                node.publish_parameter_event(data['msg'])
    except Exception as e:
        node.get_logger().error(f"Error processing message: {e}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("Connection closed")

def on_open(ws):
    subscribe_msgs = [
        {
            "op": "subscribe",
            "topic": "/scan",
            "type": "sensor_msgs/msg/LaserScan"
        },
        {
            "op": "subscribe",
            "topic": "/odom",
            "type": "nav_msgs/msg/Odometry"
        },
        {
            "op": "subscribe",
            "topic": "/parameter_events",
            "type": "rcl_interfaces/msg/ParameterEvent"
        }
    ]
    for msg in subscribe_msgs:
        ws.send(json.dumps(msg))
    print("Connection opened and subscribed to /scan, /odom, and /parameter_events topics")

def main(args=None):
    rclpy.init(args=args)
    node = ScanOdomRepublisher()

    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://192.168.64.58:9090",
                                on_open=on_open,
                                on_message=lambda ws, message: on_message(ws, message, node),
                                on_error=on_error,
                                on_close=on_close)

    ws_thread = threading.Thread(target=ws.run_forever)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ws.close()

if __name__ == "__main__":
    main()
