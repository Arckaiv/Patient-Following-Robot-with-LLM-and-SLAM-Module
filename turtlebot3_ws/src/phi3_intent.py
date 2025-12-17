#!/usr/bin/env python3
import ollama
import json
import sys
import re 
import os 

# --- Configuration ---
# Define the list of known rooms for the LLM to choose from
ROOM_LIST = ["Reception", "Emergency Room", "Laboratory", "Ward"]

# 🌟 KEY: Mapping Dictionary for LLM output and abbreviations
ROOM_MAPPING = {
    "reception": "Reception",
    "emergency room": "Emergency Room",
    "emergencyroom": "Emergency Room", 
    "er": "Emergency Room",           
    "laboratory": "Laboratory",
    "lab": "Laboratory",              
    "ward": "Ward"
}

# --- System Prompt: The most critical part ---
SYSTEM_PROMPT = f"""
You are a robotic assistant command processor. Your task is to analyze user text and classify the intent into one of two modes: FOLLOW or GUIDE.

Output must be a single, valid JSON object with two keys:
1. "intent": strictly "GUIDE" or "FOLLOW".
2. "target": The extracted destination (if intent is GUIDE) from the list {ROOM_LIST}, or "user" (if intent is FOLLOW).

DO NOT include any explanation, markdown formatting (like ```json), or text outside the JSON object.
"""

def classify_intent(user_command):
    """Sends the command to Ollama Phi-3 for classification and handles cleanup."""

    # 1. Check Ollama Service
    try:
        ollama.show("phi3") 
    except Exception:
        print("ERROR: Ollama service or Phi-3 model is not available.")
        print("Please ensure 'ollama serve' is running in another terminal.")
        return None

    # 2. Call the LLM
    print(f"-> Analyzing: '{user_command}'...")
    response = ollama.chat(
        model='phi3', 
        messages=[
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': user_command}
        ]
    )

    result_text = response['message']['content']
        
    # --- START OF ROBUST JSON GUARDRAILS ---
    
    # 1. Strip markdown fences and standard whitespace
    cleaned_text = result_text.replace("```json", "").replace("```", "").strip()
    
    # 2. Truncation Guardrail: Find the last closing brace and cut off contamination
    brace_pos = cleaned_text.rfind('}')
    if brace_pos != -1:
        cleaned_text = cleaned_text[:brace_pos + 1]
    
    # 3. Aggressive Whitespace Cleanup: Remove all internal non-essential whitespace
    cleaned_text = re.sub(r'\s+', '', cleaned_text)

    # --- END OF ROBUST JSON GUARDRAILS ---

    # 4. Final check before parsing
    if not cleaned_text.startswith('{'):
        return None 
            
    try:
        return json.loads(cleaned_text) 
    except json.JSONDecodeError as e:
        return None 

# --- NEW TERMINAL INTERFACE FUNCTIONS (These are helpers for the main logic) ---

def fallback_select_and_execute():
    """Handles the user selecting a destination or the FOLLOW intent from a list."""
    
    print("\n--- 🚨 INTENT FAILED: MANUAL OVERRIDE 🚨 ---")
    print("The robot could not determine your request. Please choose an option:")
    
    # Master List of all available commands
    MASTER_COMMANDS = {
        1: ("GUIDE", "Reception"),
        2: ("GUIDE", "Emergency Room"),
        3: ("GUIDE", "Laboratory"),
        4: ("GUIDE", "Ward"),
        5: ("FOLLOW", "user")
    }

    # Display available options
    for num, (intent, target) in MASTER_COMMANDS.items():
        if intent == "GUIDE":
            print(f"  [{num}] Guide to {target.upper()}")
        else:
            print(f"  [{num}] FOLLOW ME")

    while True:
        try:
            choice = input("\nEnter option number (1-5): ").strip()
            choice_num = int(choice)
            
            if choice_num in MASTER_COMMANDS:
                intent, target = MASTER_COMMANDS[choice_num]
                
                print(f"\n✅ Command Confirmed: {intent} to {target.upper()}")
                
                if intent == "FOLLOW":
                    print("🏃 Starting Human Following Mode.")
                else:
                    print("--- [Navigation Command Sent] ---")
                    
                return 
            else:
                print("Invalid number. Please enter a number between 1 and 5.")
        
        except ValueError:
            print("Invalid input. Please enter a number.")


def manual_destination_select():
    """
    Handles the user selecting a room destination manually (used when target is invalid).
    """
    
    print("\n--- MANUAL DESTINATION SELECTION ---")
    
    ROOM_OPTIONS = {1: "Reception", 2: "Emergency Room", 3: "Laboratory", 4: "Ward"}
    
    print("The robot could not confirm the target. Please choose a room:")
    for num, room in ROOM_OPTIONS.items():
        print(f"  [{num}] {room.upper()}")

    while True:
        try:
            choice = input("\nEnter room number (1-4): ").strip()
            choice_num = int(choice)
            
            if choice_num in ROOM_OPTIONS:
                selected_target = ROOM_OPTIONS[choice_num]
                print(f"\n✅ Target Confirmed: {selected_target.upper()}")
                print("--- [Navigation Command Sent] ---")
                return
            else:
                print("Invalid number. Please enter a number between 1 and 4.")
        
        except ValueError:
            print("Invalid input. Please enter a number.")


# --- MAIN EXECUTION LOGIC ---

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 phi3_intent.py \"<Your Command Here>\"")
        sys.exit(1)

    command_string = " ".join(sys.argv[1:]) 

    # 1. Run the classification
    intent_data = classify_intent(command_string)
    
    # CRITICAL FALLBACK CHECK: If JSON parsing failed
    if not intent_data:
        print("\n❌ LLM Failed to Classify Intent.")
        fallback_select_and_execute()
        return

    # --- Processing Valid LLM Output ---
    
    intent = intent_data.get('intent', 'N/A')
    target = intent_data.get('target', 'N/A')
    
    print("\n" + "="*40)
    print(f"🤖 User Command: {command_string}")
    print(f"🧠 LLM Classification: {intent} / {target}")
    print("="*40)

    # Validation Setup
    target_lower = target.lower()
    official_target_name = ROOM_MAPPING.get(target_lower)
    is_valid_guide_target = (official_target_name is not None)
    
    if intent == "GUIDE":
        
        # Scenario A: Target Identified (e.g., 'Laboratory' via 'lab') -> Ask for Confirmation
        if is_valid_guide_target:
            
            target_to_display = official_target_name.upper()
            
            # --- Confirmation Prompt ---
            print(f"\n❔ Is the correct destination: **{target_to_display}**?")
            
            while True:
                # 🌟 KEY MODIFICATION: Offer FOLLOW ME option explicitly
                confirmation = input("Confirm? (y/n) or Follow Me (f): ").lower().strip()
                
                if confirmation == 'y':
                    print(f"\n✅ Confirmation Success. Initiating navigation to {target_to_display}.")
                    break
                elif confirmation == 'n':
                    # User rejected the destination. Ask if they want to follow instead.
                    sub_confirmation = input("Destination rejected. Do you want the robot to FOLLOW you instead? (y/n): ").lower().strip()
                    
                    if sub_confirmation == 'y':
                        print("\n🏃 Switching to Human Following Mode.")
                    else:
                        print("❌ Moving to manual destination selection.")
                        manual_destination_select()
                    break
                    
                elif confirmation == 'f':
                    # Direct 'Follow Me' command from the confirmation prompt
                    print("\n🏃 Switching to Human Following Mode.")
                    break
                else:
                    print("Invalid input. Please type 'y' (yes), 'n' (no), or 'f' (follow).")
        
        # Scenario B: Target Unclear/Invalid/Unknown -> Go Directly to Manual Select
        else:
            print("❗ Destination unclear or invalid (Not a recognized room).")
            manual_destination_select()

    # --- FOLLOW Intent is still simple ---
    elif intent == "FOLLOW":
        print(f"\n🏃 Starting Human Following Mode. Target: {target.upper()}")

    # --- Fallback (if LLM outputted an intent we don't recognize) ---
    else:
        print("\n❌ Intent not recognized (Neither GUIDE nor FOLLOW).")
        fallback_select_and_execute()


if __name__ == "__main__":
    main()
