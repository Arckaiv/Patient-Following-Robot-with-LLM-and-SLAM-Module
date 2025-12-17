#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
# IMPORT THE NECESSARY MODULES TO GET THE ABSOLUTE PATH
from ament_index_python.packages import get_package_share_directory 
from std_srvs.srv import SetBool 

# NEW IMPORTS FOR TRACKING/POSE
from geometry_msgs.msg import PoseStamped 
import numpy as np


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # --- State Variables ---
        self.detection_enabled = True # Start enabled by default
        self.person_class_id = 0 # Assuming YOLOv8 COCO classes: 0 is 'person'

        # --- Depth/Pose Estimation Parameters (TUNE THESE!) ---
        # These are placeholders. Optimal values depend on your specific Gazebo camera model.
        self.FOCAL_LENGTH = 500.0   # Placeholder: Pixels per meter/unit
        self.REAL_PERSON_HEIGHT_M = 1.7 # meters

        # --- Service for Toggling Detection ---
        self.toggle_service = self.create_service(
            SetBool,
            '/yolo/toggle_detection', 
            self.toggle_detection_callback
        )
        self.get_logger().info('YOLO Detector toggle service is ready.')
        
        # --- Initialization ---
        self.bridge = CvBridge()
        
        # --- Model Loading ---
        pkg_share_dir = get_package_share_directory('yolo_pkg') 
        model_path = os.path.join(pkg_share_dir, 'models', 'best.onnx') 
        
        self.model = YOLO(model_path) 
        self.get_logger().info(f'YOLO model loaded successfully from: {model_path}')
        
        # --- ROS Sub/Pub ---
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Image, '/yolo/image_detections', 10)
        # NEW: Target Pose Publisher
        self.target_pub = self.create_publisher(PoseStamped, '/tracker/target_pose', 10)


    def image_callback(self, msg):
        H, W = 0, 0
        
        if not self.detection_enabled:
            # If disabled, publish the raw image and skip all inference logic
            self.publisher.publish(msg) 
            return

        # 1. Convert ROS Image to OpenCV Image (BGR format)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            H, W = cv_image.shape[:2] # Get image height and width
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # ------------------------------------------------------------------
        # 2. Run YOLO TRACKING using ByteTrack
        # ------------------------------------------------------------------
        results = self.model.track(
            cv_image, 
            persist=True, 
            tracker="bytetrack.yaml", 
            verbose=False,
            classes=[self.person_class_id] # Filter results to PERSON class (0) only
        ) 
        
        if not results or results[0].boxes.id is None:
            # No detections or no tracked objects
            self.publisher.publish(msg) 
            return 
            
        r = results[0]
        
        # --- FOCUS LOGIC: Find the target to follow ---
        
        tracked_boxes = r.boxes
        tracked_ids = tracked_boxes.id.cpu().numpy().astype(int)
        
        # Focus on the person with the LOWEST ID (the first person seen)
        min_id = np.min(tracked_ids)
        target_index = np.where(tracked_ids == min_id)[0][0]
        
        # Get the information for the chosen target
        target_box_tensor = tracked_boxes.xyxy[target_index]
        target_id = tracked_ids[target_index]
        
        x1, y1, x2, y2 = target_box_tensor.cpu().numpy().astype(int)
        target_box = (x1, y1, x2, y2)
        target_center = ((x1 + x2) // 2, (y1 + y2) // 2)

        self.get_logger().debug(f"Tracking ID: {target_id}")

        # 3. Publish Target Pose
        
        # Estimate Distance (Z-axis)
        pixel_height = y2 - y1
        if pixel_height < 10: # Avoid division by zero or very small numbers
             self.get_logger().warn("Target too small/far, skipping pose estimation.")
             return
             
        estimated_Z = (self.FOCAL_LENGTH * self.REAL_PERSON_HEIGHT_M) / pixel_height
        
        # Estimate Center X-axis offset
        center_x_offset_pixels = target_center[0] - (W / 2)
        estimated_X = (estimated_Z * center_x_offset_pixels) / self.FOCAL_LENGTH
        
        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link' # Pose is relative to camera frame
        
        pose_msg.pose.position.x = float(estimated_X) # Lateral offset
        pose_msg.pose.position.y = 0.0 
        pose_msg.pose.position.z = float(estimated_Z) # Distance forward
        
        self.target_pub.publish(pose_msg)
        
        # 4. Visualize and Publish Annotated Image
        annotated_frame = r.plot()
        
        # Highlight the specific tracked target for visualization
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 255), 4) 
        cv2.putText(annotated_frame, f'Target ID: {target_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        
        try:
            ros_output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_output_msg.header = msg.header
        except Exception as e:
            self.get_logger().error(f'cv_bridge output exception: {e}')
            return
            
        self.publisher.publish(ros_output_msg)
            
    def toggle_detection_callback(self, request, response):
        self.detection_enabled = request.data

        # Set the response for the service client
        response.success = True
        if self.detection_enabled:
            response.message = "YOLO Detection Enabled (ByteTrack Active)."
        else:
            response.message = "YOLO Detection Disabled."

        self.get_logger().info(response.message)
        return response
        
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
