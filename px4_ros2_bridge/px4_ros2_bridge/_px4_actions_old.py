#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import asyncio
from mavsdk import System
from mavsdk.telemetry import LandedState
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import threading
from px4_ros2_msgs.srv import TakeOff, GoTo

class PX4TelemetryActionsNode(Node):
    def __init__(self):
        super().__init__('px4_telemetry_and_actions_node')

        # QoS Profile for real-time performance
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

        # Initialize drone connection
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

        # Create ROS 2 services for takeoff and land
        self.takeoff_service = self.create_service(TakeOff, 'px4/takeoff', self.handle_takeoff_request)
        self.land_service = self.create_service(Trigger, 'px4/land', self.handle_land_request)
        self.arm_service = self.create_service(Trigger, 'px4/arm', self.handle_arm_request)
        self.disarm_service = self.create_service(Trigger, 'px4/disarm', self.handle_disarm_request)
        self.hold_position_service = self.create_service(Trigger, 'px4/hold_position', self.handle_hold_position_request)
        self.return_to_launch_service = self.create_service(Trigger, 'px4/return_to_launch', self.handle_return_to_launch_request)
        self.emergency_stop_service = self.create_service(Trigger, 'px4/emergency_stop', self.handle_emergency_stop_request)
        self.go_to_location_service = self.create_service(GoTo, 'px4/go_to_location', self.handle_go_to_location_request)

        # Timer to manage drone connection
        self.create_timer(2.0, self.spin_once)
        self.get_logger().info("Initialized PX4 Telemetry and Actions Node")
        
        # Store tasks for cleanup
        self.tasks = []

    def run_async_loop(self):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def spin_once(self):
        """Timer callback to manage connection attempts."""
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

    async def takeoff(self, altitude: float = 10.0):
        """Take off to specified altitude and verify."""
        try:
            self.get_logger().info(f"Taking off to {altitude}m...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            await self.wait_for_altitude(altitude)
            self.get_logger().info("Takeoff completed!")
        except Exception as e:
            self.get_logger().error(f"Takeoff failed: {str(e)}")
            raise

    async def wait_for_altitude(self, target_altitude: float, tolerance: float = 0.3):
        """Wait for drone to reach target altitude within tolerance."""
        async for position in self.drone.telemetry.position():
            if abs(position.relative_altitude_m - target_altitude) < tolerance:
                return

    async def land(self):
        """Land the drone and verify landing."""
        try:
            self.get_logger().info("Landing drone...")
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self.get_logger().warn("Drone already landed.")
                    return
                elif landed_state == LandedState.IN_AIR:
                    self.get_logger().info("Drone is IN_AIR")
                    break

            await self.drone.action.land()
            await self.wait_for_landed_state(LandedState.ON_GROUND)
            await self.disarm()
            self.get_logger().info("Landing completed! Disarmed the drone.")
        except Exception as e:
            self.get_logger().error(f"Landing failed: {str(e)}")
            raise

    async def wait_for_landed_state(self, target_state: LandedState):
        """Wait for specific landed state."""
        async for state in self.drone.telemetry.landed_state():
            if state == target_state:
                return

    async def wait_for_armed(self, is_armed: bool):
        """Wait for the drone to be armed or disarmed."""
        async for armed in self.drone.telemetry.armed():
            if armed == is_armed:
                return

    async def arm(self):
        """Arm the drone and verify."""
        try:
            self.get_logger().info("Arming drone...")
            await self.drone.action.arm()
            await self.wait_for_armed(True)
            self.get_logger().info("Drone armed!")
        except Exception as e:
            self.get_logger().error(f"Arming failed: {str(e)}")
            raise

    async def disarm(self):
        """Disarm the drone and verify."""
        try:
            self.get_logger().info("Disarming drone...")
            await self.drone.action.disarm()
            await self.wait_for_armed(False)
            self.get_logger().info("Drone disarmed!")
        except Exception as e:
            self.get_logger().error(f"Disarming failed: {str(e)}")
            raise

    async def hold_position(self):
        """Hold position at current location."""
        try:
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self.get_logger().warn("Drone is not IN_AIR.")
                    return
                elif landed_state == LandedState.IN_AIR:
                    self.get_logger().info("Drone is IN_AIR, lets hold its position.")
                    break

            self.get_logger().info("Holding position...")
            await self.drone.action.hold()
            self.get_logger().info("Position hold activated!")
        except Exception as e:
            self.get_logger().error(f"Position hold failed: {str(e)}")
            raise

    async def return_to_launch(self):
        """Return to launch position and verify landing."""
        try:
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self.get_logger().warn("Drone is not IN_AIR.")
                    return
                elif landed_state == LandedState.IN_AIR:
                    self.get_logger().info("Drone is IN_AIR, lets return to launch.")
                    break

            self.get_logger().info("Returning to launch position...")
            await self.drone.action.return_to_launch()
            await self.wait_for_landed_state(LandedState.ON_GROUND)
            await self.disarm()
            self.get_logger().info("Return to launch completed!")
        except Exception as e:
            self.get_logger().error(f"Return to launch failed: {str(e)}")
            raise

    async def emergency_stop(self):
        """Perform emergency stop/kill."""
        try:
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self.get_logger().info("Drone is not IN_AIR.")
                    break
                elif landed_state == LandedState.IN_AIR:
                    self.get_logger().warn("Drone is IN_AIR, cannot stop the drone.")
                    return

            self.get_logger().info("Executing emergency stop...")
            await self.drone.action.kill()
            await self.wait_for_armed(False)
            self.get_logger().info("Emergency stop executed!")
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {str(e)}")
            raise


    async def go_to_location(self, latitude: float = 0.0, longitude: float = 0.0, altitude: float = 30.0, yaw_deg: float =0.0):
        """Navigate to the specified location."""
        try:
            self.get_logger().info("Fetching AMSL altitude at home location...")
            async for terrain_info in self.drone.telemetry.home():
                absolute_altitude = terrain_info.absolute_altitude_m
                break

            if latitude == 0.0 or longitude == 0.0:
                async for position in self.drone.telemetry.position():
                    latitude = position.latitude_deg
                    longitude = position.longitude_deg
                    break

            self.get_logger().info("Checking if drone is in air...")
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self.get_logger().info("Drone is not in air. Cannot go to location")
                    return
                elif landed_state == LandedState.IN_AIR:
                    self.get_logger().warn("Drone is IN_AIR.")
                    break

            await asyncio.sleep(1)
            flying_alt = absolute_altitude + altitude
            self.get_logger().info(f"Navigating to latitude: {latitude}, longitude: {longitude}, altitude: {flying_alt}")
            await self.drone.action.goto_location(latitude, longitude, flying_alt, yaw_deg)
            self.get_logger().info("Reached target location!")
        except Exception as e:
            self.get_logger().error(f"Go to location failed: {str(e)}")
            raise

    def handle_go_to_location_request(self, request, response):
        """Handle service request for go_to_location."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot go to location."
            return response

        future = asyncio.run_coroutine_threadsafe(self.go_to_location(request.latitude, request.longitude, request.altitude, request.yaw_deg), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Go_to_location initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Go_to_location failed: {str(e)}"
        return response


    def handle_takeoff_request(self, request, response):
        """Handle service request for takeoff."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot take off."
            return response

        future = asyncio.run_coroutine_threadsafe(self.perform_takeoff(request.altitude), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Takeoff initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Takeoff failed: {str(e)}"
        return response

    async def perform_takeoff(self, altitude: float = 10.0):
        """Perform takeoff with pre-checks."""
        try:
            # Check if the drone is armed
            self.get_logger().info("Checking if drone is armed...")
            async for armed in self.drone.telemetry.armed():
                if not armed:
                    self.get_logger().info("Drone not armed. Arming now...")
                    await self.arm()
                    break

            await self.takeoff(altitude)
        except Exception as e:
            self.get_logger().error(f"Perform takeoff failed: {str(e)}")
            raise

    def handle_land_request(self, request, response):
        """Handle service request for landing."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot land."
            return response

        future = asyncio.run_coroutine_threadsafe(self.land(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Landing initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Landing failed: {str(e)}"
        return response

    def handle_arm_request(self, request, response):
        """Handle service request for arming."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot arm."
            return response

        future = asyncio.run_coroutine_threadsafe(self.arm(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Arming initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Arming failed: {str(e)}"
        return response

    def handle_disarm_request(self, request, response):
        """Handle service request for disarming."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot disarm."
            return response

        future = asyncio.run_coroutine_threadsafe(self.disarm(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Disarming initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Disarming failed: {str(e)}"
        return response

    def handle_hold_position_request(self, request, response):
        """Handle service request for holding position."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot hold position."
            return response

        future = asyncio.run_coroutine_threadsafe(self.hold_position(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Holding position initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Holding position failed: {str(e)}"
        return response

    def handle_return_to_launch_request(self, request, response):
        """Handle service request for return to launch."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot return to launch."
            return response

        future = asyncio.run_coroutine_threadsafe(self.return_to_launch(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Return to launch initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Return to launch failed: {str(e)}"
        return response

    def handle_emergency_stop_request(self, request, response):
        """Handle service request for emergency stop."""
        if not self.connection_status:
            response.success = False
            response.message = "Drone not connected. Cannot emergency stop."
            return response

        future = asyncio.run_coroutine_threadsafe(self.emergency_stop(), self.loop)
        try:
            future.result()
            response.success = True
            response.message = "Emergency stop initiated successfully."
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop failed: {str(e)}"
        return response



def main(args=None):
    rclpy.init(args=args)
    node = PX4TelemetryActionsNode()

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
# from mavsdk.telemetry import LandedState
# from std_srvs.srv import Trigger
# import threading

# class PX4TakeoffLandNode(Node):
#     def __init__(self):
#         super().__init__('px4_takeoff_land_node')

#         # QoS Profile for real-time performance
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Initialize drone connection
#         self.drone = System()
#         self.connection_status = False
#         self.connection_retries = 0
#         self.max_retries = 5
#         self.loop = asyncio.new_event_loop()
#         self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
#         self.async_thread.start()

#         # Create ROS 2 services for takeoff and land
#         self.takeoff_service = self.create_service(Trigger, 'takeoff', self.handle_takeoff_request)
#         self.land_service = self.create_service(Trigger, 'land', self.handle_land_request)

#         # Timer to manage drone connection
#         self.create_timer(2.0, self.spin_once)
#         self.get_logger().info("Initialized PX4 Takeoff and Land Node")

#     def run_async_loop(self):
#         """Run the asyncio event loop in a separate thread."""
#         asyncio.set_event_loop(self.loop)
#         self.loop.run_forever()

#     def spin_once(self):
#         """Timer callback to manage connection attempts."""
#         if not self.connection_status:
#             if self.connection_retries < self.max_retries:
#                 self.get_logger().info(f"Attempting to connect to drone... (Attempt {self.connection_retries + 1}/{self.max_retries})")
#                 asyncio.run_coroutine_threadsafe(self.connect_to_drone(), self.loop)
#                 self.connection_retries += 1
#             else:
#                 self.get_logger().error("Failed to connect after maximum retries. Please check if SITL is running correctly.")
#                 self.destroy_node()
#                 rclpy.shutdown()

#     async def connect_to_drone(self):
#         """Connect to the drone using SITL connection string."""
#         try:
#             connection_ports = [
#                 "udp://:14540",
#                 "udp://:14550",
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

#             self.get_logger().info("Waiting for drone connection...")
#             async for state in self.drone.core.connection_state():
#                 if state.is_connected:
#                     self.get_logger().info("Drone connected!")
#                     self.connection_status = True
#                     break
#                 await asyncio.sleep(0.1)
#         except Exception as e:
#             self.get_logger().error(f"Connection error: {str(e)}")

#     async def takeoff(self, altitude: float = 10.0):
#         """Take off to specified altitude and verify."""
#         try:
#             self.get_logger().info(f"Taking off to {altitude}m...")
#             await self.drone.action.set_takeoff_altitude(altitude)
#             await self.drone.action.takeoff()
#             await self.wait_for_altitude(altitude)
#             self.get_logger().info("Takeoff completed!")
#         except Exception as e:
#             self.get_logger().error(f"Takeoff failed: {str(e)}")
#             raise

#     async def wait_for_altitude(self, target_altitude: float, tolerance: float = 0.3):
#         """Wait for drone to reach target altitude within tolerance."""
#         async for position in self.drone.telemetry.position():
#             if abs(position.relative_altitude_m - target_altitude) < tolerance:
#                 return

#     async def land(self):
#         """Land the drone and verify landing."""
#         try:
#             self.get_logger().info("Landing drone...")
#             async for landed_state in self.drone.telemetry.landed_state():
#                 if landed_state == LandedState.ON_GROUND:
#                     self.get_logger().info("Drone already landed.")
#                     return
#                 elif landed_state == LandedState.IN_AIR:
#                     self.get_logger().info("Drone is IN_AIR")
#                     break

#             await self.drone.action.land()
#             await self.wait_for_landed_state(LandedState.ON_GROUND)
#             await self.disarm()
#             self.get_logger().info("Landing completed! Disarmed the drone.")
#         except Exception as e:
#             self.get_logger().error(f"Landing failed: {str(e)}")
#             raise

#     async def wait_for_landed_state(self, target_state: LandedState):
#         """Wait for specific landed state."""
#         async for state in self.drone.telemetry.landed_state():
#             if state == target_state:
#                 return

#     async def wait_for_armed(self, is_armed: bool):
#         """Wait for the drone to be armed or disarmed."""
#         async for armed in self.drone.telemetry.armed():
#             if armed == is_armed:
#                 return

#     async def arm(self):
#         """Arm the drone and verify."""
#         try:
#             self.get_logger().info("Arming drone...")
#             await self.drone.action.arm()
#             await self.wait_for_armed(True)
#             self.get_logger().info("Drone armed!")
#         except Exception as e:
#             self.get_logger().error(f"Arming failed: {str(e)}")
#             raise

#     async def disarm(self):
#         """Disarm the drone and verify."""
#         try:
#             self.get_logger().info("Disarming drone...")
#             await self.drone.action.disarm()
#             await self.wait_for_armed(False)
#             self.get_logger().info("Drone disarmed!")
#         except Exception as e:
#             self.get_logger().error(f"Disarming failed: {str(e)}")
#             raise

#     def handle_takeoff_request(self, request, response):
#         """Handle service request for takeoff."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot take off."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.perform_takeoff(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Takeoff initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Takeoff failed: {str(e)}"
#         return response

#     async def perform_takeoff(self, altitude: float = 10.0):
#         """Perform takeoff with pre-checks."""
#         try:
#             # Check if the drone is armed
#             self.get_logger().info("Checking if drone is armed...")
#             async for armed in self.drone.telemetry.armed():
#                 if not armed:
#                     self.get_logger().info("Drone not armed. Arming now...")
#                     await self.arm()
#                     break

#             await self.takeoff(altitude)
#         except Exception as e:
#             self.get_logger().error(f"Perform takeoff failed: {str(e)}")
#             raise

#     def handle_land_request(self, request, response):
#         """Handle service request for landing."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot land."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.land(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Landing initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Landing failed: {str(e)}"
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = PX4TakeoffLandNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()








# ############################################## TAKEOFF COMPLETED


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import asyncio
# from mavsdk import System
# from mavsdk.telemetry import LandedState
# from std_srvs.srv import Trigger
# import threading

# class PX4TakeoffLandNode(Node):
#     def __init__(self):
#         super().__init__('px4_takeoff_land_node')

#         # QoS Profile for real-time performance
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Initialize drone connection
#         self.drone = System()
#         self.connection_status = False
#         self.connection_retries = 0
#         self.max_retries = 5
#         self.loop = asyncio.new_event_loop()
#         self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
#         self.async_thread.start()

#         # Create ROS 2 services for takeoff and land
#         self.takeoff_service = self.create_service(Trigger, 'takeoff', self.handle_takeoff_request)
#         self.land_service = self.create_service(Trigger, 'land', self.handle_land_request)

#         # Timer to manage drone connection
#         self.create_timer(2.0, self.spin_once)
#         self.get_logger().info("Initialized PX4 Takeoff and Land Node")

#     def run_async_loop(self):
#         """Run the asyncio event loop in a separate thread."""
#         asyncio.set_event_loop(self.loop)
#         self.loop.run_forever()

#     def spin_once(self):
#         """Timer callback to manage connection attempts."""
#         if not self.connection_status:
#             if self.connection_retries < self.max_retries:
#                 self.get_logger().info(f"Attempting to connect to drone... (Attempt {self.connection_retries + 1}/{self.max_retries})")
#                 asyncio.run_coroutine_threadsafe(self.connect_to_drone(), self.loop)
#                 self.connection_retries += 1
#             else:
#                 self.get_logger().error("Failed to connect after maximum retries. Please check if SITL is running correctly.")
#                 self.destroy_node()
#                 rclpy.shutdown()

#     async def connect_to_drone(self):
#         """Connect to the drone using SITL connection string."""
#         try:
#             connection_ports = [
#                 "udp://:14540",
#                 "udp://:14550",
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

#             self.get_logger().info("Waiting for drone connection...")
#             async for state in self.drone.core.connection_state():
#                 if state.is_connected:
#                     self.get_logger().info("Drone connected!")
#                     self.connection_status = True
#                     break
#                 await asyncio.sleep(0.1)
#         except Exception as e:
#             self.get_logger().error(f"Connection error: {str(e)}")

#     async def takeoff(self, altitude: float = 10.0):
#         """Take off to specified altitude and verify."""
#         try:
#             self.get_logger().info(f"Taking off to {altitude}m...")
#             await self.drone.action.set_takeoff_altitude(altitude)
#             await self.drone.action.takeoff()
#             await self.wait_for_altitude(altitude)
#             self.get_logger().info("Takeoff completed!")
#         except Exception as e:
#             self.get_logger().error(f"Takeoff failed: {str(e)}")
#             raise

#     async def wait_for_altitude(self, target_altitude: float, tolerance: float = 0.3):
#         """Wait for drone to reach target altitude within tolerance."""
#         async for position in self.drone.telemetry.position():
#             if abs(position.relative_altitude_m - target_altitude) < tolerance:
#                 return

#     async def land(self):
#         """Land the drone and verify landing."""
#         try:
#             self.get_logger().info("Landing drone...")
#             await self.drone.action.land()
#             await self.wait_for_landed_state(LandedState.ON_GROUND)
#             self.get_logger().info("Landing completed!")
#         except Exception as e:
#             self.get_logger().error(f"Landing failed: {str(e)}")
#             raise

#     async def wait_for_landed_state(self, target_state: LandedState):
#         """Wait for specific landed state."""
#         async for state in self.drone.telemetry.landed_state():
#             if state == target_state:
#                 return

#     async def wait_for_armed(self, is_armed: bool):
#         """Wait for the drone to be armed or disarmed."""
#         async for armed in self.drone.telemetry.armed():
#             if armed == is_armed:
#                 return

#     async def arm(self):
#         """Arm the drone and verify."""
#         try:
#             self.get_logger().info("Arming drone...")
#             await self.drone.action.arm()
#             await self.wait_for_armed(True)
#             self.get_logger().info("Drone armed!")
#         except Exception as e:
#             self.get_logger().error(f"Arming failed: {str(e)}")
#             raise

#     def handle_takeoff_request(self, request, response):
#         """Handle service request for takeoff."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot take off."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.perform_takeoff(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Takeoff initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Takeoff failed: {str(e)}"
#         return response

#     async def perform_takeoff(self, altitude: float = 10.0):
#         """Perform takeoff with pre-checks."""
#         try:
#             # Check if the drone is armed
#             self.get_logger().info("Checking if drone is armed...")
#             async for armed in self.drone.telemetry.armed():
#                 if not armed:
#                     self.get_logger().info("Drone not armed. Arming now...")
#                     await self.arm()
#                     break

#             await self.takeoff(altitude)
#         except Exception as e:
#             self.get_logger().error(f"Perform takeoff failed: {str(e)}")
#             raise

#     def handle_land_request(self, request, response):
#         """Handle service request for landing."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot land."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.land(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Landing initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Landing failed: {str(e)}"
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = PX4TakeoffLandNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
















# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import asyncio
# from mavsdk import System
# from mavsdk.telemetry import LandedState
# from std_srvs.srv import Trigger
# import threading

# class PX4TakeoffLandNode(Node):
#     def __init__(self):
#         super().__init__('px4_takeoff_land_node')

#         # QoS Profile for real-time performance
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Initialize drone connection
#         self.drone = System()
#         self.connection_status = False
#         self.connection_retries = 0
#         self.max_retries = 5
#         self.loop = asyncio.new_event_loop()
#         self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
#         self.async_thread.start()

#         # Create ROS 2 services for takeoff and land
#         self.takeoff_service = self.create_service(Trigger, 'takeoff', self.handle_takeoff_request)
#         self.land_service = self.create_service(Trigger, 'land', self.handle_land_request)

#         # Timer to manage drone connection
#         self.create_timer(2.0, self.spin_once)
#         self.get_logger().info("Initialized PX4 Takeoff and Land Node")

#     def run_async_loop(self):
#         """Run the asyncio event loop in a separate thread."""
#         asyncio.set_event_loop(self.loop)
#         self.loop.run_forever()

#     def spin_once(self):
#         """Timer callback to manage connection attempts."""
#         if not self.connection_status:
#             if self.connection_retries < self.max_retries:
#                 self.get_logger().info(f"Attempting to connect to drone... (Attempt {self.connection_retries + 1}/{self.max_retries})")
#                 asyncio.run_coroutine_threadsafe(self.connect_to_drone(), self.loop)
#                 self.connection_retries += 1
#             else:
#                 self.get_logger().error("Failed to connect after maximum retries. Please check if SITL is running correctly.")
#                 self.destroy_node()
#                 rclpy.shutdown()

#     async def connect_to_drone(self):
#         """Connect to the drone using SITL connection string."""
#         try:
#             connection_ports = [
#                 "udp://:14540",
#                 "udp://:14550",
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

#             self.get_logger().info("Waiting for drone connection...")
#             async for state in self.drone.core.connection_state():
#                 if state.is_connected:
#                     self.get_logger().info("Drone connected!")
#                     self.connection_status = True
#                     break
#                 await asyncio.sleep(0.1)
#         except Exception as e:
#             self.get_logger().error(f"Connection error: {str(e)}")

#     async def takeoff(self, altitude: float = 10.0):
#         """Take off to specified altitude and verify."""
#         try:
#             self.get_logger().info(f"Taking off to {altitude}m...")
#             await self.drone.action.set_takeoff_altitude(altitude)
#             await self.drone.action.takeoff()
#             await self.wait_for_altitude(altitude)
#             self.get_logger().info("Takeoff completed!")
#         except Exception as e:
#             self.get_logger().error(f"Takeoff failed: {str(e)}")
#             raise

#     async def wait_for_altitude(self, target_altitude: float, tolerance: float = 0.3):
#         """Wait for drone to reach target altitude within tolerance."""
#         async for position in self.drone.telemetry.position():
#             if abs(position.relative_altitude_m - target_altitude) < tolerance:
#                 return

#     async def land(self):
#         """Land the drone and verify landing."""
#         try:
#             self.get_logger().info("Landing drone...")
#             await self.drone.action.land()
#             await self.wait_for_landed_state(LandedState.ON_GROUND)
#             self.get_logger().info("Landing completed!")
#         except Exception as e:
#             self.get_logger().error(f"Landing failed: {str(e)}")
#             raise

#     async def wait_for_landed_state(self, target_state: LandedState):
#         """Wait for specific landed state."""
#         async for state in self.drone.telemetry.landed_state():
#             if state == target_state:
#                 return

#     def handle_takeoff_request(self, request, response):
#         """Handle service request for takeoff."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot take off."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.takeoff(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Takeoff initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Takeoff failed: {str(e)}"
#         return response

#     def handle_land_request(self, request, response):
#         """Handle service request for landing."""
#         if not self.connection_status:
#             response.success = False
#             response.message = "Drone not connected. Cannot land."
#             return response

#         future = asyncio.run_coroutine_threadsafe(self.land(), self.loop)
#         try:
#             future.result()
#             response.success = True
#             response.message = "Landing initiated successfully."
#         except Exception as e:
#             response.success = False
#             response.message = f"Landing failed: {str(e)}"
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = PX4TakeoffLandNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
