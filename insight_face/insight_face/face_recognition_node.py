import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from datetime import datetime
import numpy as np
from insightface.app import FaceAnalysis
from ament_index_python.packages import get_package_share_directory
from glob import glob
from tqdm import tqdm

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__("face_recognition_node")

        # ROS2 topic for camera feed
        self.declare_parameter('camera_topic_name', 'camera/id_70001/image_raw')
        self.declare_parameter('recognition_threshold', 0.7)
        self.camera_topic_name = self.get_parameter('camera_topic_name').get_parameter_value().string_value
        self.recognition_threshold = self.get_parameter('recognition_threshold').get_parameter_value().double_value
        self.subscription = self.create_subscription(Image, self.camera_topic_name, self.image_callback, 10)
        self.recognition_publisher = self.create_publisher(String, 'recognition_info', 10)
        self.bridge = CvBridge()
        
        # Initialize face recognition app
        self.app = FaceAnalysis(name='buffalo_l')
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        
        # Directory for storing reference faces
        self.reference_faces_dir = os.path.join(get_package_share_directory('insight_face'), 'random_person')
        
        # Load embeddings from the reference folder
        self.names, self.embeddings = self.load_reference_faces(self.reference_faces_dir)
        self.get_logger().info("FaceRecognitionNode initialized.")

    def load_reference_faces(self, folder):
        """Load and generate averaged embeddings from images in the folder."""
        names = []
        embeddings = []
        person_embeddings = {}

        self.get_logger().info(f"Checking directory {folder}")
        img_paths = glob(f'{folder}/*')
        
        for img_path in tqdm(img_paths):
            person_name = os.path.basename(img_path).split('_')[1]
            img = cv2.imread(img_path)
            if img is None:
                continue
            faces = self.app.get(img)
            if len(faces) != 1:
                continue
            face = faces[0]
            # Store embeddings for each person
            if person_name not in person_embeddings:
                person_embeddings[person_name] = []
            person_embeddings[person_name].append(face.normed_embedding)
        
        # Average the embeddings for each person
        for person_name, face_embeddings in person_embeddings.items():
            names.append(person_name)
            avg_embedding = np.mean(face_embeddings, axis=0)  # Averaging the embeddings
            embeddings.append(avg_embedding)
        
        embeddings = np.stack(embeddings, axis=0)
        self.get_logger().info(f"Loaded reference faces: {', '.join(names)}")
        return names, embeddings

    def recognize_face(self, input_img):
        """Recognize the face in the input image and compare it with stored embeddings."""
        faces = self.app.get(input_img)
        if len(faces) != 1:
            return "No face or multiple faces detected"
        
        detected_embedding = faces[0].normed_embedding
        scores = np.dot(detected_embedding, self.embeddings.T)  # Compare to known embeddings
        scores = np.clip(scores, 0., 1.)
        idx = np.argmax(scores)
        max_score = scores[idx]
        
        threshold = self.recognition_threshold
        if max_score >= threshold:
            recognized_name = self.names[idx]
            rec_pub_data = String()
            rec_pub_data.data = str(recognized_name)
            self.recognition_publisher.publish(rec_pub_data)
            return (f"Face recognized as {recognized_name} with a confidence score of {max_score:.2f}", recognized_name, max_score)
        else:
            return ("Face not recognized", None, None)

    def image_callback(self, msg):
        """Callback to process incoming image and recognize the face."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result_str, recognized_name, conf_level = self.recognize_face(cv_image)
        
        if recognized_name is not None:
            display_name = f"{recognized_name}, {round(conf_level*100, 2)}%"
        else:
            display_name = "Unknown"

        # Draw bounding box and label if a face is recognized
        faces = self.app.get(cv_image)
        for face in faces:
            bbox = list(map(int, face.bbox))
            cv2.rectangle(cv_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
            cv2.putText(cv_image, display_name, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        # Log result_str
        log_entries = [f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {result_str}"]
        write_log_file = f"{os.path.join(get_package_share_directory('insight_face'), 'log')}/face_detection_log.txt"
        with open(write_log_file, "a") as log_file:
            for entry in log_entries:
                log_file.write(entry + "\n")

        # Display the image with result_str
        # cv2.imshow("Face Recognition", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()










# import os
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from datetime import datetime
# import numpy as np
# from insightface.app import FaceAnalysis
# from ament_index_python.packages import get_package_share_directory

# class FaceRecognitionNode(Node):
#     def __init__(self):
#         super().__init__("face_recognition_node")

#         self.declare_parameter('camera_topic_name', 'camera/id_70001/image_raw')
#         self.camera_topic_name = self.get_parameter('camera_topic_name').get_parameter_value().string_value

#         self.subscription = self.create_subscription(Image, self.camera_topic_name, self.image_callback, 10)
#         self.bridge = CvBridge()
#         self.app = FaceAnalysis()
#         self.app.prepare(ctx_id=0, det_size=(640, 640))
#         self.reference_faces_dir = os.path.join(get_package_share_directory('insight_face'), 'random_person')

#         self.reference_faces = self.load_reference_faces(self.reference_faces_dir)
#         self.get_logger().info("FaceRecognitionNode initialized.")


#     def load_reference_faces(self, folder):
#         """Load reference faces from subfolders in the installed directory."""
#         references = {}
#         self.get_logger().info(f"Checking directory {folder}")
#         if os.path.isdir(folder):
#             self.get_logger().info(f"Loading reference faces from {folder}")        
#             for image_file in os.listdir(folder):
#                 person_dir_split = os.path.splitext(image_file)
#                 person_name_split = person_dir_split[0].split('_')
#                 # self.get_logger().info(str(person_name_split[0] + '_' + person_name_split[1]))
#                 self.get_logger().info(f"looping through {image_file}")
#                 references[person_name_split[1]] = []
#                 img_path = os.path.join(folder, image_file)
#                 # Read the image file using OpenCV
#                 img = cv2.imread(img_path)
#                 if img is not None:
#                     faces = self.app.get(img)  # Assuming this is a method to extract faces
#                     # If faces are found, add the first face embedding to the references
#                     if faces:
#                         references[person_name_split[1]].append(faces[0].embedding)
            
#         self.get_logger().info(f"Loaded references for: {', '.join(references.keys())}")
#         return references

#     def old_load_reference_faces(self, folder):
#         """Load reference faces from subfolders."""
#         references = {}
#         self.get_logger().info(folder)
#         for person_folder in os.listdir(folder):
#             person_path = os.path.join(folder, person_folder)
#             if os.path.isdir(person_path):
#                 references[person_folder] = []
#                 for image_file in os.listdir(person_path):
#                     img_path = os.path.join(person_path, image_file)
#                     img = cv2.imread(img_path)
#                     if img is not None:
#                         faces = self.app.get(img)
#                         if faces:
#                             references[person_folder].append(faces[0].embedding)
#         self.get_logger().info(f"Loaded references for: {', '.join(references.keys())}")
#         return references

#     def find_closest_match(self, face_embedding):
#         """Find the closest match for a detected face."""
#         best_match = ("Unknown", float("inf"))
#         for name, embeddings in self.reference_faces.items():
#             for ref_embedding in embeddings:
#                 distance = np.linalg.norm(face_embedding - ref_embedding)
#                 if distance < best_match[1]:
#                     best_match = (name, distance)
#         return best_match

#     def image_callback(self, msg):
#         """Process incoming images."""
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         faces = self.app.get(cv_image)
#         log_entries = []

#         for face in faces:
#             name, distance = self.find_closest_match(face.embedding)
#             if distance > 1.0:  # Distance threshold for recognition
#                 name = "Unknown"
#             bbox = list(map(int, face.bbox))
#             cv2.rectangle(cv_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
#             cv2.putText(cv_image, name, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

#             # Logging
#             timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#             log_entries.append(f"{timestamp}, {name}, Distance: {distance:.2f}, BBox: {bbox}")

#         # Write log to file
#         write_log_file = f"{os.path.join(get_package_share_directory('insight_face'), 'log')}/face_detection_log.txt"
#         with open(write_log_file, "a") as log_file:
#             for entry in log_entries:
#                 log_file.write(entry + "\n")

#         # Display the image
#         cv2.imshow("Face Recognition", cv_image)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FaceRecognitionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
