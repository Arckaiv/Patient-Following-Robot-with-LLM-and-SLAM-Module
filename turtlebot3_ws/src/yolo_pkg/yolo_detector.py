#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool 

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # --- State Variables ---
        self.detection_enabled = False # Initial state is OFF
        self.subscription = None       # Handle for the subscription (starts inactive)
        
        # --- Toggle Service ---
        self.toggle_service = self.create_service(
            SetBool,
            '/yolo/toggle_detection', 
            self.toggle_detection_callback
        )
        self.get_logger().info('YOLO Detector toggle service is ready and in OFF mode.')
        
        # --- Initialization ---
        self.bridge = CvBridge()
        
        # --- Model Loading ---
        try:
            pkg_share_dir = get_package_share_directory('yolo_pkg') 
            model_path = os.path.join(pkg_share_dir, 'models', 'best.onnx')  
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded successfully from: {model_path}')
        except Exception as e:
             self.get_logger().error(f"Failed to load YOLO model: {e}")
             # It's better to continue without the model than crash the node
             self.model = None 
        
        # --- ROS Publisher (Always active to potentially send status or annotated image) ---
        self.publisher = self.create_publisher(Image, '/yolo/image_detections', 10)

        # CRITICAL: We DO NOT create the subscription here. It is managed in the callback.

    def image_callback(self, msg):
        """
        Processes images only when the subscription is active (i.e., detection is ON).
        The guardrail check for detection_enabled is no longer needed here.
        """
        if self.model is None:
             self.get_logger().error("Model not loaded. Cannot process image.")
             return

        # 1. Convert ROS Image to OpenCV Image (BGR format)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # 2. Run YOLO Inference
        results = self.model(cv_image, verbose=False)  
        
        if not results:
            annotated_frame = cv_image 
        else:
            r = results[0]  
            # 3. Visualize Results
            annotated_frame = r.plot()  
        
        # 4. Convert Annotated OpenCV Image back to ROS Image
        try:
            ros_output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_output_msg.header = msg.header
        except Exception as e:
            self.get_logger().error(f'cv_bridge output exception: {e}')
            return
            
        # 5. Publish the Annotated Image
        self.publisher.publish(ros_output_msg)
            
    def toggle_detection_callback(self, request, response):
        """
        Dynamically creates or destroys the image subscription to manage the ON/OFF state.
        """
        new_state = request.data
        
        if new_state == self.detection_enabled:
            # State is already requested state.
            response.success = True
            response.message = f"YOLO Detection already {'Enabled' if new_state else 'Disabled'}."
            self.get_logger().warn(response.message)
            return response

        if new_state: # Requested ON (True)
            # Create the subscription to start receiving image data
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)
            self.get_logger().info('Subscription activated. Detection starting.')
            
        else: # Requested OFF (False)
            # Destroy the subscription to stop image processing and publishing
            if self.subscription is not None:
                self.destroy_subscription(self.subscription)
                self.subscription = None
            self.get_logger().info('Subscription destroyed. Detection stopped.')

        self.detection_enabled = new_state
        response.success = True
        response.message = f"YOLO Detection {'Enabled.' if new_state else 'Disabled.'}"
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
