#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import os
import sys
import json
import re
import ollama
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool # New import for the service call

# --- MAPPING CONFIGURATION ---
# The names the LLM is allowed to generate (e.g., in the SYSTEM_PROMPT)
ROOM_LIST = ["Room 1", "Room 2", "Room 3", "Room 4", "Room 5", "Room 6", "Consultation Room", "Lobby"]

# 🌟 KEY: Mapping Dictionary for LLM output (Target) to YAML map keys (ROS Destination)
# Left: LLM Output (lower-cased) | Right: YAML Key (used by self.goals)
INTENT_TO_YAML_MAP = {
    "room1": "room_1", 
    "room2": "room_2", 
    "room3": "room_3", 
    "room4": "room_4", 
    "room5": "room_5",  # <--- THIS FIXES YOUR ISSUE
    "room6": "room_6",
    "consultation room": "consultation_room",
    "consultation": "consultation_room",
    "lobby": "lobby",
}

# --- System Prompt: The most critical part ---
# Find and replace your existing SYSTEM_PROMPT at the top of the file
SYSTEM_PROMPT = f"""
You are a robotic assistant command processor. Your task is to analyze user text and classify the intent into one of three modes: FOLLOW, GUIDE, or STOP.

Output must be a single, valid JSON object with two keys:
1. "intent": strictly "GUIDE", "FOLLOW", or "STOP".
2. "target": The extracted destination (if intent is GUIDE) from the list {ROOM_LIST}, or "user" (if intent is FOLLOW), or "robot" (if intent is STOP).

DO NOT include any explanation, markdown formatting (like ```json), or text outside the JSON object.
"""

class IntegratedNavigator(Node):
    def __init__(self):
        super().__init__('integrated_navigator_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = self.load_goals()

        # Check Ollama connection
        try:
            ollama.show("phi3")
            self.get_logger().info("✅ Connected to Ollama (Phi-3).")
        except Exception:
            self.get_logger().error("❌ Ollama service or Phi-3 model is not available. Run 'ollama serve'.")
            sys.exit(1)
        self.toggle_client = self.create_client(SetBool, '/yolo/toggle_detection')
        while not self.toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('YOLO toggle service not available, waiting...')
        self.get_logger().info('✅ YOLO toggle service available.')
        # 1. Ensure FOLLOW detection starts OFF
        self.toggle_follow_detection(False) # Set initial state
            
    def load_goals(self):
        """Load room coordinates from YAML file."""
        try:
            pkg_share = get_package_share_directory('smart_brain')
            # Check for the YAML file name
            yaml_path = os.path.join(pkg_share, 'config', 'map_locations.yaml') 
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load map_locations.yaml: {e}")
            return {}

    def classify_intent(self, user_command):
        """Sends the command to Ollama Phi-3 for classification and handles cleanup."""
        print(f"-> Analyzing: '{user_command}'...")

        try:
            response = ollama.chat(
                model='phi3',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': user_command}
                ]
            )
            result_text = response['message']['content']
        except Exception as e:
            self.get_logger().error(f"Ollama Chat Error: {e}")
            return None

        # --- ROBUST JSON GUARDRAILS (YOUR EXISTING LOGIC) ---
        cleaned_text = result_text.replace("```json", "").replace("```", "").strip()
        brace_pos = cleaned_text.rfind('}')
        if brace_pos != -1: cleaned_text = cleaned_text[:brace_pos + 1]
        cleaned_text = re.sub(r'\s+', '', cleaned_text)
        
        if not cleaned_text.startswith('{'): return None
        
        try:
            return json.loads(cleaned_text)
        except json.JSONDecodeError:
            return None

    # --- Navigation Execution Function ---
    def execute_navigation(self, yaml_key):
        """The actual ROS 2 Navigation Command"""
        if yaml_key not in self.goals:
            self.get_logger().error(f"❌ Key '{yaml_key}' missing from map_locations.yaml. Cannot navigate.")
            return

        print(f"🚀 Sending Robot to [{yaml_key}]...")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 Action Server not available! Is Nav2 running?")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        target_data = self.goals[yaml_key]
        
        # Assign coordinates from YAML
        goal_msg.pose.pose.position.x = float(target_data['x'])
        goal_msg.pose.pose.position.y = float(target_data['y'])
        
        # Assign Quaternion Orientation
        goal_msg.pose.pose.orientation.z = float(target_data.get('z', 0.0))
        goal_msg.pose.pose.orientation.w = float(target_data['w'])
        goal_msg.pose.pose.orientation.x = 0.0 
        goal_msg.pose.pose.orientation.y = 0.0 

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ Goal was rejected by the navigation stack.")
            return

        print("⏳ Moving...")
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        status = res_future.result().status
        if status == 4:
            print(f"✅ Arrived at {yaml_key}!")
        else:
            print(f"❌ Navigation failed (Status: {status}).")

    # --- Fallback/Manual Interaction Functions (Your Logic) ---
    def fallback_select_and_execute(self):
        """Handles the user selecting a destination or the FOLLOW intent from a list."""
        
        print("\n--- 🚨 INTENT FAILED: MANUAL OVERRIDE 🚨 ---")
        print("The robot could not determine your request. Please choose an option:")
        
        # Create sorted list of unique display names for manual selection
        display_names = sorted(list(set(INTENT_TO_YAML_MAP.keys())))

        room_options = {i + 1: name for i, name in enumerate(display_names)}
        
        # Add FOLLOW option
        next_index = len(room_options) + 1
        room_options[next_index] = "FOLLOW"

        for num, target in room_options.items():
            if target == "FOLLOW":
                print(f"  [{num}] FOLLOW ME")
            else:
                print(f"  [{num}] Guide to {target.upper()}")
        
        while True:
            try:
                choice = input("\nEnter option number: ").strip().lower()
                
                # Check for direct 'follow' command
                if choice in ['f', 'follow']:
                    print("\n🏃 Starting Human Following Mode.")
                    return # Exit loop
                    
                choice_num = int(choice)
                
                if choice_num in room_options:
                    target = room_options[choice_num]
                    
                    if target == "FOLLOW":
                        print("\n🏃 Starting Human Following Mode.")
                    else:
                        yaml_key = INTENT_TO_YAML_MAP[target]
                        print(f"\n✅ Command Confirmed: GUIDE to {target.upper()}")
                        self.execute_navigation(yaml_key) # Navigation Call
                    return # Exit loop
                else:
                    print(f"Invalid number. Please enter a number between 1 and {next_index}.")
            except ValueError:
                print("Invalid input. Please enter a number.")

    def manual_destination_select(self):
        """Handles the user selecting a room destination manually (used when target is invalid)."""
        
        print("\n--- MANUAL DESTINATION SELECTION ---")
        display_names = sorted([name for name in INTENT_TO_YAML_MAP.keys() if name != 'entrance'])
        ROOM_OPTIONS = {i + 1: name for i, name in enumerate(display_names)}
        
        print("The robot could not confirm the target. Please choose a room:")
        for num, room in ROOM_OPTIONS.items():
            print(f"  [{num}] {room.upper()}")

        while True:
            try:
                choice = input("\nEnter room number: ").strip()
                choice_num = int(choice)
                
                if choice_num in ROOM_OPTIONS:
                    selected_target = ROOM_OPTIONS[choice_num]
                    yaml_key = INTENT_TO_YAML_MAP[selected_target]
                    
                    print(f"\n✅ Target Confirmed: {selected_target.upper()}")
                    self.execute_navigation(yaml_key) # Navigation Call
                    return
                else:
                    print(f"Invalid number. Please enter a number between 1 and {len(ROOM_OPTIONS)}.")
            except ValueError:
                print("Invalid input. Please enter a number.")
                
    def toggle_follow_detection(self, state_bool):
        """
        Calls the /yolo/toggle_detection service to turn detection ON/OFF.
        state_bool: True to turn ON, False to turn OFF.
        """
        request = SetBool.Request()
        request.data = state_bool
        
        future = self.toggle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            status = "ON" if state_bool else "OFF"
            self.get_logger().info(f'🏃 YOLO Detection successfully turned {status}.')
            return True
        else:
            self.get_logger().error('❌ Failed to call YOLO toggle service.')
            return False

    def stop_robot_and_follow(self):
        """Cancels any active navigation goal and turns OFF follow detection."""
        # 1. Cancel Navigation Goal (Important if robot is currently navigating)
        # Assuming you use the cancel_goal method from the ActionClient (which requires the goal handle)
        # For simplicity, we'll just log a stop command here, but in a full implementation, you'd send a cancel signal.
        # For now, we'll just log and assume the velocity is stopped by Nav2 failure or a subsequent command.
        
        # If the robot is actively moving to a room, you should cancel the goal.
        # This requires storing the active goal handle, which is complex.
        self.get_logger().warn("⚠️ Stopping Robot - Active navigation goal assumed cancelled.")
        
        # 2. Turn OFF Follow Detection
        self.toggle_follow_detection(False)
        print("\n🛑 Robot stopped and Human Following is disabled.")

    def run_main_loop(self):
        """The main interactive loop for the node."""
        print(f"🤖 Smart Navigator Online. Valid Destinations: {list(INTENT_TO_YAML_MAP.keys())}")

        while True:
            cmd = input("\n🗣️  Command (or 'q' to quit, 's' to stop): ").strip() # Added 's' hint
            if cmd == 'q':
                break
            
            # --- MANUAL STOP KEY CHECK ---
            if cmd.lower() == 's':
                self.stop_robot_and_follow()
                continue # Skip LLM analysis and go back to prompt
                
            # 1. Get Intent
            intent_data = self.classify_intent(cmd)
            
            # CRITICAL FALLBACK CHECK: If JSON parsing failed
            if not intent_data:
                print("\n❌ LLM Failed to Classify Intent.")
                self.fallback_select_and_execute()
                continue

            # --- Processing Valid LLM Output ---
            intent = intent_data.get('intent', 'N/A')
            target = intent_data.get('target', 'N/A')
            target_lower = target.lower()
            
            yaml_key = INTENT_TO_YAML_MAP.get(target_lower)
            is_valid_guide_target = (yaml_key is not None)
            
            print("\n" + "="*40)
            print(f"🤖 User Command: {cmd}")
            print(f"🧠 LLM Classification: {intent} / {target}")
            print("="*40)

            if intent == "GUIDE":
                if is_valid_guide_target:
                    self.toggle_follow_detection(False) # Guide mode means no follow
                    target_to_display = target.upper()
                    
                    # --- Confirmation Prompt (Your Custom Logic) ---
                    print(f"\n❔ Is the correct destination: **{target_to_display}**?")
                    while True:
                        confirmation = input("Confirm? (y/n) or Follow Me (f): ").lower().strip()
                        
                        if confirmation == 'y':
                            print(f"\n✅ Confirmation Success. Initiating navigation to {target_to_display}.")
                            self.execute_navigation(yaml_key) # Navigation Call
                            break
                        elif confirmation == 'n':
                            sub_confirmation = input("Destination rejected. Do you want the robot to FOLLOW you instead? (y/n): ").lower().strip()
                            if sub_confirmation == 'y':
                                print("\n🏃 Switching to Human Following Mode.")
                            else:
                                print("❌ Moving to manual destination selection.")
                                self.manual_destination_select()
                            break
                        elif confirmation == 'f':
                            print("\n🏃 Switching to Human Following Mode.")
                            break
                        else:
                            print("Invalid input. Please type 'y' (yes), 'n' (no), or 'f' (follow).")
                
                else:
                    print(f"❗ Destination '{target}' unclear or invalid.")
                    self.manual_destination_select()

            elif intent == "FOLLOW":
                print(f"\n🏃 Starting Human Following Mode. Target: {target.upper()}")
                self.toggle_follow_detection(True)
                # You would add your FOLLOW implementation call here
            
            elif intent == "STOP":
                self.stop_robot_and_follow()
            
            else:
                print("\n❌ Intent not recognized (Neither GUIDE nor FOLLOW).")
                self.fallback_select_and_execute()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigator()
    
    try:
        node.run_main_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Navigator shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
