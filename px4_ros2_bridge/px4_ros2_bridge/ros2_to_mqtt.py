import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

class DataHandler(Node):
    def __init__(self):
        super().__init__('data_handler')

        self.declare_parameter('mqtt_broker_ip_address', '192.168.1.21')
        self.declare_parameter('mqtt_broker_port_address', 1883)
        self.declare_parameter('drone_id', '70001')
        self.mqtt_broker_ip_address = self.get_parameter('mqtt_broker_ip_address').get_parameter_value().string_value
        self.mqtt_broker_port_address = self.get_parameter('mqtt_broker_port_address').get_parameter_value().integer_value
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        mqtt_client_name = f'data_handler_{self.drone_id}'
        self.mqtt_client = mqtt.Client(mqtt_client_name)
        self.mqtt_client.on_connect = self.on_connect
        self.connected = False
        self.reconnect_interval = 5  # Reconnect every 5 seconds if not connected
        self.connect_to_broker()
        self.get_logger().info('Initialized ROS2 to MQTT Publisher node')

        # Configure QoS profile for better real-time performance
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.px4_battery_subs = self.create_subscription(BatteryState, 'px4/battery', self.battery_callback, qos_profile=qos_profile)
        self.px4_flight_mode_subs = self.create_subscription(String, 'px4/flight_mode', self.flight_mode_callback, qos_profile=qos_profile)
        self.px4_position_subs = self.create_subscription(NavSatFix, 'px4/position', self.position_callback, qos_profile=qos_profile)
        self.px4_velocity_subs = self.create_subscription(TwistStamped, 'px4/velocity', self.velocity_callback, qos_profile=qos_profile)
        self.px4_device_network_status_subs = self.create_subscription(String, 'px4/device_network_status', self.device_network_status_callback, qos_profile=qos_profile)

    def connect_to_broker(self):
        """Try to connect to the MQTT broker and keep retrying if the connection fails."""
        while not self.connected:
            try:
                self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker_ip_address}:{self.mqtt_broker_port_address}")
                self.mqtt_client.connect(self.mqtt_broker_ip_address, self.mqtt_broker_port_address, 60)
                self.mqtt_client.loop_start()  # Start the non-blocking network loop
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f"Failed to connect to broker: {str(e)}")
                self.get_logger().info(f"Retrying connection in {self.reconnect_interval} seconds...")
                time.sleep(self.reconnect_interval)

    def on_connect(self, client, userdata, flags, rc):
        """Callback for when the MQTT client connects to the broker."""
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker with result code {str(rc)}")
            self.connected = True
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with result code {str(rc)}")
            self.connected = False
            self.connect_to_broker()  # Try to reconnect if the connection fails

    def mqtt_publish(self, msg):
        try:
            if not self.mqtt_client.is_connected():
                self.get_logger().warn("MQTT connection is lost. Retrying to connect to broker and then send the message.")
                if self.connect_to_broker():
                    self.get_logger().info('[MQTT_PUB]: Connection restabilished. Sending the message...')
                else:
                    self.get_logger().info('[MQTT_PUB]: Connection Failed. Try relaunching and check MQTT connection')

            # Parse the message from the ROS 2 bridge
            data = json.loads(msg.data)
            qos = data['qos']
            retain = data['retain']
            mqtt_topic = data['topic']
            mqtt_data = json.dumps(data['data'])

            # Publish to the MQTT broker
            pubback = self.mqtt_client.publish(topic=mqtt_topic, payload=mqtt_data, qos=qos, retain=retain)
            # Check the result of the publish
            # pubback_status = self.check_pubback_resp(pubback.rc)
        except Exception as e:
            self.get_logger().error(f"Error in MQTT publish: {str(e)}")

    def publish_to_mqtt_topic(self, parent_topic, data, qos, retain):
        # /robot/{vuid}
        """Publish the data to the outbound MQTT ROS topic."""
        topic_name = f'drone/{self.drone_id}/{parent_topic}'
        payload = json.dumps({"topic": topic_name, "data": data, "qos": qos, "retain": retain})
        mqtt_msg = String(data=payload)
        self.mqtt_publish(mqtt_msg)
        # self.get_logger().info(f"Published topic - {topic_name}")

    def battery_callback(self, msg):
        payload = {
            'voltage': msg.voltage,
            'current': msg.current,
            'percentage': msg.percentage
        }
        self.publish_to_mqtt_topic("battery", payload, 0, False)

    def flight_mode_callback(self, msg):
        payload = {'mode': msg.data}
        self.publish_to_mqtt_topic("flight_mode", payload, 0, False)
    
    def position_callback(self, msg):
        payload = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        self.publish_to_mqtt_topic("position", payload, 0, False)
    
    def velocity_callback(self, msg):
        payload = {
            'linear_x': msg.twist.linear.x,
            'linear_y': msg.twist.linear.y,
            'linear_z': msg.twist.linear.z,
            'angular_x': msg.twist.angular.x,
            'angular_y': msg.twist.angular.y,
            'angular_z': msg.twist.angular.z
        }
        self.publish_to_mqtt_topic("velocity", payload, 0, False)
    
    def device_network_status_callback(self, msg):
        device_network_data = json.loads(msg.data)
        payload = {
            'system_ip_address': device_network_data.get("system_ip_address", None),
            'network_status': device_network_data.get("network_status", None),
            'latency': device_network_data.get("latency", None),
            'packet_loss': device_network_data.get("packet_loss", None),
            'signal_strength': device_network_data.get("signal_strength", None)
        }
        self.publish_to_mqtt_topic("device_network", payload, 0, False)

def main(args=None):
    rclpy.init(args=args)
    data_handler = DataHandler()
    rclpy.spin(data_handler)
    data_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
