import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import Jetson.GPIO as GPIO
import os
from ament_index_python.packages import get_package_share_directory
from glob import glob
from tqdm import tqdm

class TriggerLedNode(Node):
    def __init__(self):
        super().__init__('trigger_led_node')

        self.declare_parameter('led_gpio_pin_1', 11)
        self.declare_parameter('led_gpio_pin_2', 13)
        self.led_gpio_pin_1 = self.get_parameter('led_gpio_pin_1').get_parameter_value().integer_value
        self.led_gpio_pin_2 = self.get_parameter('led_gpio_pin_2').get_parameter_value().integer_value

        # Set up GPIO pins for LEDs on Jetson devices
        self.setup_gpio_config()

        # Load reference faces
        self.reference_faces_dir = os.path.join(get_package_share_directory('insight_face'), 'random_person')
        self.reference_list = self.load_reference_faces(self.reference_faces_dir)
        self.updated_recognized_list = []
        
        self.led_blink_status = 0        # 0: no in use, 1: in use
        self.gpio_trigger_cb = None
        self.led_counter = 0

        # Subscriber to the face recognition topic
        self.create_subscription(String, 'recognition_info', self.listener_callback, 10)

    # def setup_gpio_config(self):
    #     self.led_pins = [self.led_gpio_pin_1, self.led_gpio_pin_2]
    #     GPIO.setmode(GPIO.BOARD)  # Use BOARD numbering
    #     GPIO.setup(self.led_pins, GPIO.OUT, initial=GPIO.LOW)
    #     self.get_logger().info(f'Configuring led pin number {str(self.led_gpio_pin_1)} and {str(self.led_gpio_pin_2)} for operation.')

    def load_reference_faces(self, folder):
        """Load and generate averaged embeddings from images in the folder."""
        self.get_logger().info(f"Checking directory {folder}")
        img_paths = glob(f'{folder}/*')
        names = []
        
        for img_path in tqdm(img_paths):
            person_name = os.path.basename(img_path).split('_')[1]
            # Store embeddings for each person
            if person_name not in names:
                names.append(person_name)
        
        self.get_logger().info(f"Loaded reference face list: {', '.join(names)}")
        return names

    def listener_callback(self, msg):
        """Callback when a recognized face name is received."""
        recognized_name = msg.data

        # Check if the recognized name exists in the reference list
        if recognized_name in self.reference_list and recognized_name not in self.updated_recognized_list:
            if self.led_blink_status:
                self.get_logger().info("LED is currently being used. Found another person in detection, skipping LED action until 10 seconds to complete.")
            else:
                # If match is found, trigger the LED flashing
                self.get_logger().info(f"Recognized: {recognized_name}")
                self.flash_leds()
                self.updated_recognized_list.append(recognized_name)

    def flash_leds(self):
        """Flash the LEDs for 10 seconds."""
        self.get_logger().info("Match found! Flashing LEDs for 10 seconds.")
        self.gpio_trigger_cb = self.create_timer(1.0, self.gpio_callback)
        self.led_blink_status = True

    def gpio_callback(self):
        # After 10 seconds, stop flashing and turn off LEDs
        self.led_counter = self.led_counter + 1
        if self.led_counter >= 10:
            GPIO.output(self.led_pins, GPIO.LOW)
            self.get_logger().info("Stopped flashing LEDs.")
            self.led_counter = 0
            self.led_blink_status = False
            self.gpio_trigger_cb.cancel()
        else:
            self.get_logger().info(f'Blinking led for remaining {str(10 - self.led_counter)} seconds')
            if (self.led_counter % 2) == 0:
                # Turn LEDs on
                GPIO.output(self.led_pins, GPIO.HIGH)
            else:
                # Turn LEDs off
                GPIO.output(self.led_pins, GPIO.LOW)

    def destroy_node(self):
        """Clean up before shutting down."""
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriggerLedNode()

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
# import time
# # import Jetson.GPIO as GPIO  # Change to RPi.GPIO if edge device is RPi
# import os
# from ament_index_python.packages import get_package_share_directory
# from glob import glob
# from tqdm import tqdm

# class TriggerLedNode(Node):
#     def __init__(self):
#         super().__init__('trigger_led_node')

#         self.declare_parameter('led_gpio_pin_1', 11)
#         self.declare_parameter('led_gpio_pin_2', 13)
#         self.led_gpio_pin_1 = self.get_parameter('led_gpio_pin_1').get_parameter_value().integer_value
#         self.led_gpio_pin_2 = self.get_parameter('led_gpio_pin_2').get_parameter_value().integer_value

#         # Set up GPIO pins for LEDs on Jetson devices
#         # self.setup_gpio_config()

#         # Load reference faces
#         self.reference_faces_dir = os.path.join(get_package_share_directory('insight_face'), 'random_person')
#         self.reference_list = self.load_reference_faces(self.reference_faces_dir)
#         self.updated_recognized_list = []
        
#         self.led_blink_status = 0        # 0: no in use, 1: in use
#         self.gpio_trigger_cb = None
#         self.led_counter = 0

#         # Subscriber to the face recognition topic
#         self.create_subscription(String, 'recognition_info', self.listener_callback, 10)

#     # def setup_gpio_config(self):
#     #     GPIO.setmode(GPIO.BOARD)  # Use BOARD numbering
#     #     self.led_pins = [self.led_gpio_pin_1, self.led_gpio_pin_2]
#     #     GPIO.setup(self.led_pins, GPIO.OUT, initial=GPIO.LOW)
#     #     self.get_logger().info(f'Configuring led pin number {str(self.led_gpio_pin_1)} and {str(self.led_gpio_pin_2)} for operation.')

#     def load_reference_faces(self, folder):
#         """Load and generate averaged embeddings from images in the folder."""
#         self.get_logger().info(f"Checking directory {folder}")
#         img_paths = glob(f'{folder}/*')
#         names = []
        
#         for img_path in tqdm(img_paths):
#             person_name = os.path.basename(img_path).split('_')[1]
#             # Store embeddings for each person
#             if person_name not in names:
#                 names.append(person_name)
        
#         self.get_logger().info(f"Loaded reference face list: {', '.join(names)}")
#         return names

#     def listener_callback(self, msg):
#         """Callback when a recognized face name is received."""
#         recognized_name = msg.data

#         # Check if the recognized name exists in the reference list
#         if recognized_name in self.reference_list and recognized_name not in self.updated_recognized_list:
#             if self.led_blink_status:
#                 self.get_logger().info("LED is currently being used. Found another person in detection, skipping LED action until 10 seconds to complete.")
#             else:
#                 # If match is found, trigger the LED flashing
#                 self.get_logger().info(f"Recognized: {recognized_name}")
#                 self.flash_leds()
#                 self.updated_recognized_list.append(recognized_name)

#     def flash_leds(self):
#         """Flash the LEDs for 10 seconds."""
#         self.get_logger().info("Match found! Flashing LEDs for 10 seconds.")
#         self.gpio_trigger_cb = self.create_timer(1.0, self.gpio_callback)
#         self.led_blink_status = True

#     def gpio_callback(self):
#         # After 10 seconds, stop flashing and turn off LEDs
#         self.led_counter = self.led_counter + 1
#         if self.led_counter >= 10:
#             self.get_logger().info("Stopped flashing LEDs.")
#             self.led_counter = 0
#             self.led_blink_status = False
#             self.gpio_trigger_cb.cancel()
#         else:
#             self.get_logger().info(f'Blinking led for remaining {str(10 - self.led_counter)} seconds')        

#     # def destroy_node(self):
#     #     """Clean up before shutting down."""
#     #     GPIO.cleanup()
#     #     super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = TriggerLedNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



























# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import time
# # import Jetson.GPIO as GPIO  # Change to RPi.GPIO if edge device is RPi
# import os
# from ament_index_python.packages import get_package_share_directory
# from glob import glob
# from tqdm import tqdm

# class TriggerLedNode(Node):
#     def __init__(self):
#         super().__init__('trigger_led_node')

#         # # Set up GPIO pins for LEDs on Jetson devices
#         # GPIO.setmode(GPIO.BOARD)  # Use BOARD numbering (change to BCM if needed)
#         # self.led_pins = [11, 13]  # Use GPIO pins 11 and 13 for LED flashing (you can change these pins)
#         # GPIO.setup(self.led_pins, GPIO.OUT, initial=GPIO.LOW)

#         # Load reference faces
#         self.reference_faces_dir = os.path.join(get_package_share_directory('insight_face'), 'random_person')
#         self.reference_list = self.load_reference_faces(self.reference_faces_dir)
#         self.updated_recognized_list = []
#         # Subscriber to the face recognition topic
#         self.create_subscription(String, 'recognition_info', self.listener_callback, 10)


#     def load_reference_faces(self, folder):
#         """Load and generate averaged embeddings from images in the folder."""
#         self.get_logger().info(f"Checking directory {folder}")
#         img_paths = glob(f'{folder}/*')
#         names = []
        
#         for img_path in tqdm(img_paths):
#             person_name = os.path.basename(img_path).split('_')[1]
#             # Store embeddings for each person
#             if person_name not in names:
#                 names.append(person_name)
        
#         self.get_logger().info(f"Loaded reference face list: {', '.join(names)}")
#         return names

#     def listener_callback(self, msg):
#         """Callback when a recognized face name is received."""
#         recognized_name = msg.data

#         # Check if the recognized name exists in the reference list
#         if recognized_name in self.reference_list and recognized_name not in self.updated_recognized_list:
#             # If match is found, trigger the LED flashing
#             self.get_logger().info(f"Recognized: {recognized_name}")
#             self.flash_leds()
#             self.updated_recognized_list.append(recognized_name)

#     def flash_leds(self):
#         """Flash the LEDs for 10 seconds."""
#         self.get_logger().info("Match found! Flashing LEDs for 10 seconds.")

#         # # Wait for 1 second before starting to flash
#         # time.sleep(1)

#         # start_time = time.time()
#         # while time.time() - start_time < 10:
#         #     # Turn LEDs on
#         #     GPIO.output(self.led_pins, GPIO.HIGH)
#         #     time.sleep(0.5)
#         #     # Turn LEDs off
#         #     GPIO.output(self.led_pins, GPIO.LOW)
#         #     time.sleep(0.5)

#         # # After 10 seconds, stop flashing and turn off LEDs
#         # GPIO.output(self.led_pins, GPIO.LOW)
#         self.get_logger().info("Stopped flashing LEDs.")

#     def destroy_node(self):
#         """Clean up before shutting down."""
#         # GPIO.cleanup()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = TriggerLedNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
