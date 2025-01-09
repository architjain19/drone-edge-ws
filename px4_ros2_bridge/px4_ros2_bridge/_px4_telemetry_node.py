#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import asyncio
from mavsdk import System
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped
from std_msgs.msg import String, Float32
import threading

class PX4TelemetryNode(Node):
    def __init__(self):
        super().__init__('px4_telemetry_node')
        
        # Configure QoS profile for better real-time performance
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize publishers
        self.position_pub = self.create_publisher(NavSatFix, 'px4/position', qos_profile)
        self.velocity_pub = self.create_publisher(TwistStamped, 'px4/velocity', qos_profile)
        self.battery_pub = self.create_publisher(BatteryState, 'px4/battery', qos_profile)
        self.flight_mode_pub = self.create_publisher(String, 'px4/flight_mode', qos_profile)
        
        # Initialize drone with modified connection parameters
        self.drone = System()
        self.connection_status = False
        self.telemetry_started = False
        self.connection_retries = 0
        self.max_retries = 5
        
        # Create event loop for async operations
        self.loop = asyncio.new_event_loop()
        
        # Start the async loop in a separate thread
        self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.async_thread.start()
        
        # Create timer for checking connection
        self.create_timer(2.0, self.spin_once)
        self.get_logger().info("Initialized PX4 Telemetry Node")
        
        # Store tasks for cleanup
        self.tasks = []

    def run_async_loop(self):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def spin_once(self):
        """Timer callback to check connection and start telemetry if needed."""
        if not self.connection_status:
            if self.connection_retries < self.max_retries:
                self.get_logger().info(f"Attempting to connect to drone... (Attempt {self.connection_retries + 1}/{self.max_retries})")
                asyncio.run_coroutine_threadsafe(self.connect_to_drone(), self.loop)
                self.connection_retries += 1
            else:
                self.get_logger().error("Failed to connect after maximum retries. Please check if SITL is running correctly.")
                self.destroy_node()
                rclpy.shutdown()
        elif self.connection_status and not self.telemetry_started:
            self.get_logger().info("Starting telemetry streams...")
            future = asyncio.run_coroutine_threadsafe(self.start_telemetry(), self.loop)
            self.telemetry_started = True

    async def connect_to_drone(self):
        """Connect to the drone using SITL connection string."""
        try:
            # Try different ports commonly used by PX4 SITL
            connection_ports = [
                "udp://:14540",
                "udp://:14550",
                "udp://127.0.0.1:14540",
                "udp://127.0.0.1:14550"
            ]
            
            for port in connection_ports:
                try:
                    self.get_logger().info(f"Trying to connect on {port}")
                    await self.drone.connect(system_address=port)
                    break
                except Exception as e:
                    self.get_logger().warning(f"Failed to connect on {port}: {str(e)}")
                    continue
            
            self.get_logger().info("Waiting for drone connection...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info("Drone connected!")
                    self.connection_status = True
                    break
                await asyncio.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Connection error: {str(e)}")
            return False
        return True

    async def start_telemetry(self):
        """Start all telemetry streams."""
        try:
            # Create tasks for each telemetry stream
            self.tasks = [
                asyncio.create_task(self.run_telemetry_loop(self.position_loop())),
                asyncio.create_task(self.run_telemetry_loop(self.velocity_loop())),
                asyncio.create_task(self.run_telemetry_loop(self.battery_loop())),
                asyncio.create_task(self.run_telemetry_loop(self.flight_mode_loop()))
            ]
            
            # Wait for all tasks to complete (they won't unless there's an error)
            await asyncio.gather(*self.tasks, return_exceptions=True)
            
        except Exception as e:
            self.get_logger().error(f"Error starting telemetry: {str(e)}")

    async def run_telemetry_loop(self, generator):
        """Helper method to run telemetry generators."""
        try:
            async for data in generator:
                # self.get_logger().info(f"Processing is done within the generator for {generator}")
                pass  # The processing is done within the generator
        except Exception as e:
            self.get_logger().error(f"Telemetry loop error: {str(e)}")
            self.get_logger().error(f"Processing failed within the generator for {generator}")

    async def position_loop(self):
        """Publish position data."""
        async for position in self.drone.telemetry.position():
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.latitude = position.latitude_deg
            msg.longitude = position.longitude_deg
            msg.altitude = position.absolute_altitude_m
            self.position_pub.publish(msg)
            yield position

    async def velocity_loop(self):
        """Publish velocity data."""
        async for velocity in self.drone.telemetry.velocity_ned():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = velocity.north_m_s
            msg.twist.linear.y = velocity.east_m_s
            msg.twist.linear.z = -velocity.down_m_s
            self.velocity_pub.publish(msg)
            yield velocity

    async def battery_loop(self):
        """Publish battery data."""
        async for battery in self.drone.telemetry.battery():
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = battery.voltage_v
            msg.percentage = battery.remaining_percent / 100.0
            self.battery_pub.publish(msg)
            yield battery

    async def flight_mode_loop(self):
        """Publish flight mode data."""
        async for flight_mode in self.drone.telemetry.flight_mode():
            msg = String()
            msg.data = str(flight_mode)
            self.flight_mode_pub.publish(msg)
            yield flight_mode

def main(args=None):
    rclpy.init(args=args)
    node = PX4TelemetryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up tasks
        if hasattr(node, 'tasks'):
            for task in node.tasks:
                task.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

























# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import asyncio
# from mavsdk import System
# from sensor_msgs.msg import NavSatFix, BatteryState
# from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped
# from std_msgs.msg import String, Float32
# import threading

# class PX4TelemetryNode(Node):
#     def __init__(self):
#         super().__init__('px4_telemetry_node')
        
#         # Configure QoS profile for better real-time performance
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
        
#         # Initialize publishers
#         self.position_pub = self.create_publisher(NavSatFix, 'px4/position', qos_profile)
#         self.attitude_pub = self.create_publisher(QuaternionStamped, 'px4/attitude', qos_profile)
#         self.velocity_pub = self.create_publisher(TwistStamped, 'px4/velocity', qos_profile)
#         self.battery_pub = self.create_publisher(BatteryState, 'px4/battery', qos_profile)
#         self.flight_mode_pub = self.create_publisher(String, 'px4/flight_mode', qos_profile)
#         self.heading_pub = self.create_publisher(Float32, 'px4/heading', qos_profile)
        
#         # Initialize drone with modified connection parameters
#         self.drone = System()
#         self.connection_status = False
#         self.telemetry_started = False
#         self.connection_retries = 0
#         self.max_retries = 5
        
#         # Create event loop for async operations
#         self.loop = asyncio.new_event_loop()
        
#         # Start the async loop in a separate thread
#         self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
#         self.async_thread.start()
        
#         # Create timer for checking connection with longer interval
#         self.create_timer(2.0, self.spin_once)
#         self.get_logger().info("Initialized PX4 Telemetry Node")

#     def run_async_loop(self):
#         """Run the asyncio event loop in a separate thread."""
#         asyncio.set_event_loop(self.loop)
#         self.loop.run_forever()

#     def spin_once(self):
#         """Timer callback to check connection and start telemetry if needed."""
#         if not self.connection_status:
#             if self.connection_retries < self.max_retries:
#                 self.get_logger().info(f"Attempting to connect to drone... (Attempt {self.connection_retries + 1}/{self.max_retries})")
#                 asyncio.run_coroutine_threadsafe(self.connect_to_drone(), self.loop)
#                 self.connection_retries += 1
#             else:
#                 self.get_logger().error("Failed to connect after maximum retries. Please check if SITL is running correctly.")
#                 self.destroy_node()
#                 rclpy.shutdown()
#         elif self.connection_status and not self.telemetry_started:
#             self.get_logger().info("Starting telemetry streams...")
#             asyncio.run_coroutine_threadsafe(self.start_telemetry(), self.loop)
#             self.telemetry_started = True

#     async def connect_to_drone(self):
#         """Connect to the drone using SITL connection string."""
#         try:
#             # Try different ports commonly used by PX4 SITL
#             connection_ports = [
#                 "udp://:14540",  # Default SITL port
#                 "udp://:14550",  # Alternative SITL port
#                 "udp://127.0.0.1:14540",
#                 "udp://127.0.0.1:14550"
#             ]
            
#             for port in connection_ports:
#                 try:
#                     self.get_logger().info(f"Trying to connect on {port}")
#                     await self.drone.connect(system_address=port)
#                     break
#                 except Exception as e:
#                     self.get_logger().warning(f"Failed to connect on {port}: {str(e)}")
#                     continue
            
#             # Wait for connection
#             self.get_logger().info("Waiting for drone connection...")
#             async for state in self.drone.core.connection_state():
#                 if state.is_connected:
#                     self.get_logger().info("Drone connected!")
#                     self.connection_status = True
#                     break
#                 await asyncio.sleep(0.1)  # Add small delay between connection checks
#         except Exception as e:
#             self.get_logger().error(f"Connection error: {str(e)}")
#             return False
#         return True

#     async def start_telemetry(self):
#         """Start all telemetry streams."""
#         try:
#             # Start all telemetry tasks
#             tasks = [
#                 self.position_loop(),
#                 self.attitude_loop(),
#                 self.velocity_loop(),
#                 self.battery_loop(),
#                 self.flight_mode_loop()
#             ]
#             await asyncio.gather(*tasks)
#         except Exception as e:
#             self.get_logger().error(f"Error starting telemetry: {str(e)}")

#     # [Rest of the telemetry loop methods remain the same]
#     async def position_loop(self):
#         """Publish position data."""
#         async for position in self.drone.telemetry.position():
#             msg = NavSatFix()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "map"
#             msg.latitude = position.latitude_deg
#             msg.longitude = position.longitude_deg
#             msg.altitude = position.absolute_altitude_m
#             self.position_pub.publish(msg)

#     async def attitude_loop(self):
#         """Publish attitude data."""
#         async for attitude in self.drone.telemetry.attitude_quaternion():
#             msg = QuaternionStamped()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "base_link"
#             msg.quaternion.w = attitude.w
#             msg.quaternion.x = attitude.x
#             msg.quaternion.y = attitude.y
#             msg.quaternion.z = attitude.z
#             self.attitude_pub.publish(msg)

#             # Also publish heading
#             heading_msg = Float32()
#             euler = await self.drone.telemetry.attitude_euler()
#             heading_msg.data = euler.yaw_deg
#             self.heading_pub.publish(heading_msg)

#     async def velocity_loop(self):
#         """Publish velocity data."""
#         async for velocity in self.drone.telemetry.velocity_ned():
#             msg = TwistStamped()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "base_link"
#             msg.twist.linear.x = velocity.north_m_s
#             msg.twist.linear.y = velocity.east_m_s
#             msg.twist.linear.z = -velocity.down_m_s
#             self.velocity_pub.publish(msg)

#     async def battery_loop(self):
#         """Publish battery data."""
#         async for battery in self.drone.telemetry.battery():
#             msg = BatteryState()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.voltage = battery.voltage_v
#             msg.percentage = battery.remaining_percent / 100.0
#             self.battery_pub.publish(msg)

#     async def flight_mode_loop(self):
#         """Publish flight mode data."""
#         async for flight_mode in self.drone.telemetry.flight_mode():
#             msg = String()
#             msg.data = str(flight_mode)
#             self.flight_mode_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PX4TelemetryNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()