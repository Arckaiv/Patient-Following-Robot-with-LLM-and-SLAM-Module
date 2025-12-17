#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = self.load_goals()

    def load_goals(self):
        """Load room coordinates from YAML file."""
        try:
            pkg_share = get_package_share_directory('smart_brain')
            yaml_path = os.path.join(pkg_share, 'config', 'map_locations.yaml')
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load locations: {e}")
            return {}

    def send_goal(self, room_name):
        """Send goal and WAIT until finished."""
        if room_name not in self.goals:
            print(f"Error: {room_name} not found in map_locations.yaml")
            return

        print(f"Waiting for navigation server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("Action server not available!")
            return

        # 1. Setup the Goal Message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        data = self.goals[room_name]
        goal_msg.pose.pose.position.x = float(data['x'])
        goal_msg.pose.pose.position.y = float(data['y'])
        goal_msg.pose.pose.orientation.w = float(data['w'])
        goal_msg.pose.pose.orientation.z = float(data.get('z', 0.0)) # Handle Z rotation if present

        print(f"Requesting navigation to {room_name}...")
        
        # 2. Send Goal and WAIT for acceptance
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("Goal was rejected by the navigation stack.")
            return

        print("Goal accepted! Robot is moving...")

        # 3. WAIT for the Result (Blocking)
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        # 4. Process Result
        result = get_result_future.result()
        status = result.status
        
        # STATUS_SUCCEEDED is 4
        if status == 4:
            print(f"✅ Arrived at {room_name}!")
        else:
            print(f"❌ Navigation failed with status code: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    
    try:
        while True:
            print("\n--- Room Selector ---")
            # Sort keys for consistent menu order
            for room in sorted(node.goals.keys()):
                print(f"- {room}")
            print("x: Exit")
            
            choice = input("Enter room name: ").strip()
            
            if choice == 'x':
                print("Exiting...")
                break
            
            # This call will now BLOCK until the robot arrives
            node.send_goal(choice)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
