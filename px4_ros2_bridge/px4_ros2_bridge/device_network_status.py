import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re
import json

class NetworkStatusPublisher(Node):
    def __init__(self):
        super().__init__('network_status_publisher')

        self.declare_parameter('system_ip_address', '192.168.1.21')
        self.system_ip_address = self.get_parameter('system_ip_address').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, 'px4/device_network_status', 10)
        self.timer = self.create_timer(5.0, self.publish_network_status)  # Publish every 5 seconds

    def get_latency_and_packet_loss(self):
        try:
            # Example: Use 'ping' to check latency
            result = subprocess.run(['ping', '-c', '4', '8.8.8.8'], capture_output=True, text=True)
            if result.returncode == 0:
                output = result.stdout
                # Extract packet loss percentage
                packet_loss = re.search(r'(\d+)% packet loss', output)
                packet_loss = int(packet_loss.group(1)) if packet_loss else 100

                avg_latency = float(output.splitlines()[-1].split('=')[-1].split('/')[1])
                return avg_latency, packet_loss
            else:
                return float('inf'), 100  # No network
        except Exception as e:
            self.get_logger().error(f"Error getting network status: {e}")
            return float('inf'), 100  # No network

    def old_get_latency_and_packet_loss(self):
        try:
            # Ping to Google's public DNS
            result = subprocess.run(
                ['ping', '-c', '4', '8.8.8.8'], capture_output=True, text=True
            )
            if result.returncode == 0:
                output = result.stdout
                # Extract packet loss percentage
                packet_loss = re.search(r'(\d+)% packet loss', output)
                packet_loss = int(packet_loss.group(1)) if packet_loss else 100

                # Extract average latency
                latency = re.search(r'avg = ([\d.]+)/', output)
                avg_latency = float(latency.group(1)) if latency else float('inf')

                return avg_latency, packet_loss
            else:
                return float('inf'), 100  # No network
        except Exception as e:
            self.get_logger().error(f"Error getting latency and packet loss: {e}")
            return float('inf'), 100

    def get_signal_strength(self):
        try:
            # Use iwconfig to fetch signal strength
            result = subprocess.run(
                ['iwconfig'], capture_output=True, text=True
            )
            if result.returncode == 0:
                output = result.stdout
                # Extract signal level (RSSI in dBm)
                signal_level = re.search(r'Signal level=(-?\d+) dBm', output)
                signal_strength = int(signal_level.group(1)) if signal_level else None
                return signal_strength
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error getting signal strength: {e}")
            return None

    def classify_network_status(self, latency, packet_loss, signal_strength):
        if packet_loss == 100 or latency == float('inf'):
            return "No Network"
        elif signal_strength and signal_strength < -70:
            return "Bad (Weak Signal)"
        elif latency < 50 and packet_loss < 10:
            return "Good"
        elif latency < 100 and packet_loss < 30:
            return "Moderate"
        else:
            return "Bad"

    def publish_network_status(self):
        latency, packet_loss = self.get_latency_and_packet_loss()
        signal_strength = self.get_signal_strength()
        status = self.classify_network_status(latency, packet_loss, signal_strength)
        ip_add = self.system_ip_address

        payload_msg = json.dumps({"system_ip_address": ip_add, "network_status": status, "latency": latency, "packet_loss": packet_loss, "signal_strength": signal_strength})
        # msg.data = f"Network Status: {status} | Latency: {latency} ms | Packet Loss: {packet_loss}% | Signal Strength: {signal_strength} dBm"

        msg = String(data=payload_msg)
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NetworkStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import subprocess

# class NetworkStatusPublisher(Node):
#     def __init__(self):
#         super().__init__('network_status_publisher')
#         self.publisher_ = self.create_publisher(String, 'network_status', 10)
#         self.timer = self.create_timer(5.0, self.publish_network_status)  # Publish every 5 seconds

#     def get_network_status(self):
#         try:
#             # Example: Use 'ping' to check latency
#             result = subprocess.run(['ping', '-c', '4', '8.8.8.8'], capture_output=True, text=True)
#             if result.returncode == 0:
#                 output = result.stdout
#                 avg_latency = float(output.splitlines()[-1].split('=')[-1].split('/')[1])
#                 if avg_latency < 50:
#                     return "Good"
#                 elif avg_latency < 100:
#                     return "Moderate"
#                 else:
#                     return "Bad"
#             else:
#                 return "No Network"
#         except Exception as e:
#             self.get_logger().error(f"Error getting network status: {e}")
#             return "Error"

#     def publish_network_status(self):
#         status = self.get_network_status()
#         msg = String()
#         msg.data = f"Network Status: {status}"
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published: {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NetworkStatusPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
