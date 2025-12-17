# Patient Following Robot
This project demonstrates an intelligent system for a TurtleBot3 (burger_cam) robot operating within a simulated hospital environment. The system uses a Local Large Language Model (LLM) to translate complex user intent into concrete navigation goals for the ROS 2 Nav2 stack, offering both autonomous guidance and human-following capabilities.
The speech-to-text process is skipped to avoid unecessary complications in the voice-command system, instead a text-based system is used in the terminal to simulate the voice-command system. This is suffice to test the LLM's capabilites to detect the intent of a string of text.

## Author
1. `Ho Xiang (A181576)`
2. `Tan Kai Ze (A202383)`
3. `Aizad Haiqal Bin Aiman Hakim SAW (A201962)`
4. `Ramnathan A/L Senthil Kumar (A203579)`

## Requirements:

### Computer Vision Component (Ultralytics and related depenencies)

**1. Navigate to your workspace**\
`cd ~/turtlebot3_ws`

**2. Remove the cached build configuration**\
`rm -rf build/CMakeCache.txt`

**2.5. (Optional but recommended) Remove full build/install folders for a clean slate**\
`rm -rf build/ install/ log/`

**3. Update package lists**\
`sudo apt update`

**4. Install Python 3.9 and its development headers**\
`sudo apt install python3.9 python3.9-dev`

**5. Install dependencies explicitly for Python 3.9**\
`python3.9 -m pip install ultralytics`\
`python3.9 -m pip install numpy==1.24.4`\
`python3.9 -m pip install onnx onnxruntime-gpu`

**6. Navigate to source directory**\
`cd ~/turtlebot3_ws/src`

**7. Clone the autorace package (contains the camera node)**\
`git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git`

**8. Return to workspace root and build**\
`cd ~/turtlebot3_ws && colcon build --symlink-install`

**9. Source the setup file**\
`echo 'export GAZEBO_PLUGIN_PATH=$HOME/turtlebot3_ws/build/turtlebot3_gazebo:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc`

**10. Setting Turtlebot3 model (burger_cam)**\
`echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc`

**11 Verify the camera feed**\
`ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py`
`rqt` (2nd Terminal)

↑ Source: https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving

### LLM Interface (Ollama Phi-3 Mini)

**1. Install Ollama**\
`curl -fsSL https://ollama.com/install.sh | sh`

**2. Download the Model**\
`ollama pull phi3`

**3. Install Python Client**\
`pip3 install ollama`

**4. Check if the model is available**\
`ollama list`

**5. Return to workspace root, build and source**\
`cd ~/turtlebot3_ws && colcon build --symlink-install`
`source install/setup.bash`


## Instructions
**LAUNCH HOSPITAL WORLD with BURGER_CAM IN GAZEBO (1ST TERMINAL)**\
`ros2 launch turtlebot3_gazebo hosp_world.launch.py use_sim_time:=True`

**RUN NAV2 STACK (2ND TERMINAL)**\
`ros2 launch smart_brain burger_cam_nav2.launch.py`

**RUN RQT TO VIEW YOLO_DETECTIONS (3RD TERMINAL)**\
`rqt`

**RUN THE CLI TO START INTENT IDENTIFICATION (4TH TERMINAL)**\
`ros2 run smart_brain integrated_navigator`
