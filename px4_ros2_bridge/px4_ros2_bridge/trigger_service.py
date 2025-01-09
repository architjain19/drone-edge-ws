import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import paho.mqtt.client as mqtt
import json
import time
from px4_ros2_msgs.srv import TakeOff, GoTo, UploadMission

class TriggerService(Node):
    def __init__(self):
        super().__init__('trigger_service_node')

        # Declare and read parameters
        self.declare_parameter('mqtt_broker_ip_address', '192.168.1.21')
        self.declare_parameter('mqtt_broker_port_address', 1883)
        self.declare_parameter('drone_id', '70001')
        self.mqtt_broker_ip_address = self.get_parameter('mqtt_broker_ip_address').get_parameter_value().string_value
        self.mqtt_broker_port_address = self.get_parameter('mqtt_broker_port_address').get_parameter_value().integer_value
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.drone_command_topic = f'drone/{self.drone_id}/drone_command'
        
        # Initialize ROS 2 service clients
        self.start_streaming_client = self.create_client(Trigger, 'px4/start_streaming')
        self.stop_streaming_client = self.create_client(Trigger, 'px4/stop_streaming')
        self.land_client = self.create_client(Trigger, 'px4/land')
        self.arm_client = self.create_client(Trigger, 'px4/arm')
        self.disarm_client = self.create_client(Trigger, 'px4/disarm')
        self.hold_position_client = self.create_client(Trigger, 'px4/hold_position')
        self.return_to_launch_client = self.create_client(Trigger, 'px4/return_to_launch')
        self.emergency_stop_client = self.create_client(Trigger, 'px4/emergency_stop')
        self.start_mission_client = self.create_client(Trigger, 'px4/start_mission')
        self.stop_mission_client = self.create_client(Trigger, 'px4/stop_mission')
        self.clear_mission_client = self.create_client(Trigger, 'px4/clear_mission')
        self.takeoff_client = self.create_client(TakeOff, 'px4/takeoff')
        self.go_to_location_client = self.create_client(GoTo, 'px4/go_to_location')
        self.upload_mission_client = self.create_client(UploadMission, 'px4/upload_mission')
        self.upload_and_start_mission_client = self.create_client(UploadMission, 'px4/upload_and_start_mission')

        # Map commands to their respective clients
        self.service_clients = {
            'start_streaming': self.start_streaming_client,
            'stop_streaming': self.stop_streaming_client,
            'land': self.land_client,
            'arm': self.arm_client,
            'disarm': self.disarm_client,
            'hold_position': self.hold_position_client,
            'return_to_launch': self.return_to_launch_client,
            'emergency_stop': self.emergency_stop_client,
            'start_mission': self.start_mission_client,
            'stop_mission': self.stop_mission_client,
            'clear_mission': self.clear_mission_client,
            'takeoff': self.takeoff_client,
            'go_to_location': self.go_to_location_client,
            'upload_mission': self.upload_mission_client,
            'upload_and_start_mission': self.upload_and_start_mission_client,
        }

        mqtt_client_name = f'mqtt_subs_{self.drone_id}'
        self.mqtt_client = mqtt.Client(mqtt_client_name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.connected = False
        self.reconnect_interval = 5  # Reconnect every 5 seconds if not connected
        self.connect_to_broker()

        self.get_logger().info('Initialized MQTT subscriber node')

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
            # Subscribe to drone command topic
            self.mqtt_client.subscribe(self.drone_command_topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {self.drone_command_topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with result code {str(rc)}")
            self.connected = False
            self.connect_to_broker()  # Try to reconnect if the connection fails

    def map_drone_action_services(self, command, params):
        self.get_logger().info(f'Received command >> {command} with params >> {params}')

        if command in self.service_clients:
            client = self.service_clients[command]
            if client.wait_for_service(timeout_sec=5.0):
                request = self.prepare_request(command, params)
                future = client.call_async(request)
                future.add_done_callback(self.handle_response)
            else:
                self.get_logger().error(f'Service {command} is not available.')
        else:
            self.get_logger().error(f'Command {command} not recognized.')

    def prepare_request(self, command, params):
        if command in ['start_streaming', 'stop_streaming', 'land', 'arm', 'disarm', 'hold_position', 'return_to_launch', 'emergency_stop', 'start_mission', 'stop_mission', 'clear_mission']:  # For Trigger-based services
            return Trigger.Request()
        elif command == 'takeoff':  # For TakeOff service
            request = TakeOff.Request()
            request.altitude = float(params.get('altitude', 10.0))
            return request
        elif command == 'go_to_location':  # For GoTo service
            request = GoTo.Request()
            request.latitude = float(params.get('latitude', 0.0))
            request.longitude = float(params.get('longitude', 0.0))
            request.altitude = float(params.get('altitude', 10.0))
            request.yaw_deg = float(params.get('yaw_deg', 0.0))
            return request
        elif command in ['upload_mission', 'upload_and_start_mission']:   # For UploadMission service
            request = UploadMission.Request()
            request.mission_file_path = str(params.get('mission_file_path', ''))
            return request
        else:
            raise ValueError(f'Unknown command: {command}')

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service call succeeded: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def on_message(self, client, userdata, message):
        """Callback for when a message is received from an MQTT topic."""
        try:
            msg_payload = message.payload.decode('utf-8')
            # self.get_logger().info(f"Received message from topic '{message.topic}': {msg_payload}")
            conv_payload = json.loads(msg_payload)

            if 'drone_command' in message.topic and conv_payload.get("target_device", None) == "edge":
                if conv_payload.get("command", None) is not None and conv_payload.get("params", None) is not None:
                    self.map_drone_action_services(conv_payload.get("command"), conv_payload.get("params"))

        except Exception as e:
            self.get_logger().error(f"Error decoding message: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the MQTT subscriber node
    trigger_service = TriggerService()

    try:
        # Keep the node spinning
        rclpy.spin(trigger_service)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up when done
        trigger_service.mqtt_client.loop_stop()  # Stop the network loop
        trigger_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()