# ROS2 project for an autonomous tractor safety system

This repository contains a ROS2 project for an autonomous tractor safety system, which relies on radar and camera sensors for object detection. The aim of this project was to create a "Version 0" grade system, which can be improved and developed in the future. Versions 0, 1 and 2 are explained below.

This project is done as part of a Master's Thesis work in the University of Oulu.

Component       | Version 0     | Version 1     | Version 2     |
----------------|------------------------------|-------------------------------|--------------------------|
Camera Node     | Working through simulation | Working with real hardware | Working with real hardware
Radar Node      | Working through simulation | Working with real hardware | Working with real hardware
Fusion Node     | Perform simple fusion based on detection timestamp and positions. Publish detections from single sensors if not associated with any other sensor detection. | Target propabilities added, publish detections from single sensors only if propability of the target is high enough. Simple target recognition using the camera machine vision system. | Utilize filtering to identify false detections more accurately (for example Kalman filtering). Improved object recognition algorithm using radar point-cloud data to verify detections.
Safety Monitor  | Adjust the tractor speed, if obstacles are detected in the vision systems sight. | Adjust tractor speed when obstacles are detected in a determined distance from the tractors path, taking into account the tractor steering angle. | Adjust tractor path if needed according to detected objects.
Tractor Control | Receives ControlCommands, but doesn't forward them | Able to control tractor speed, but steering angle comes straight from AgOpenGPS | Able to fully control the tractor based on AgOpenGPS and the safety system.
AgOpenGPS Node  | Not implemented | Translates AgOpenGPS commands to ROS2 messages, and forwads them to the safety monitor node. | Communicates with AgOpenGPS, being able to adjust the map and assist in the path planning process.
Simulation and testing | Tested only through simulations | Tested also in real-world environments | Tested also in real-world environments
All components included | Functional safety in simulation. Simulated speed adjustment. | Functional safety with real hardware. Tractor speed adjustment. | Machine vision system assists in the control and path planning process providing details of the surroundings, such as obstacles in the real world. The path of the vehicle will be adjusted according to this updated map.


## System architecture:g

![System architecture](system_architecture.png)

### Camera node
- Connects to the camera using depth-ai
- Currently based on Luxonis OAK-d s2 message format
- Camera data is transformed into *CameraDetection*s
    - Messages include:
        - Header
            - Timestamp
            - Target_ID
        - Results
            - A list of hypotheses for the object
            - Hypotheses are of ROS type ObjectHypothesis
            - Includes:
                - Class_id: A unique ID of the object class.
                - Score: The confidence value of the detected object in range 0-1
                - Not used in Version 0 but will be used in Version 1.
        - Position
            - X, y and z in meters
        - Is_tracking
            - Boolean value for determining if the object is being tracked or not.
            - Not used in Version 0 but will be used in Version 1.
        - Tracking_id
            - A unique id for a tracked object.
            - Not used in Version 0 but will be used in Version 1.
        - Bbox
            - A bounding box surrounding the object.
            - Of type BoundingBox2D
            - Not used in Version 0 but can be utilized later
- Subscribes to topic “/oakd/detections”
    - This is the topic where oak-d s2 will publish its detections
- Publishes camera data in the ROS topic “/camera_detections”
### Radar node
- Nodes for both UART and CAN connections
- Radar data is transformed into RadarDetection type ROS-messages
    - Messages include:
        - Position
            - x, y and z in meters
        - Speed
        - Header
            - Timestamp
            - Target_ID
        - Distance
            - Calculated from the position points, based on basic trigonometry
- Publishes radar data in the ROS topic “/radar_detections”
### Fusion node
- Subscribes to topics “/camera_detections” and “/radar_detections”
- Performs sensor fusion for camera and radar
- Publishes the processed detections in the ROS topic “/fused_detections”
### AgOpenGPS node
- Communication from AgOpenGPS to ROS2
- Translates the AgOpenGPS messages to ROS messages
- Publishes in the ROS topic “/control/agopen”
- NOT IMPLEMENTED YET!
### Tractor control node
- Sends control signals to the tractor through CAN
- Subscribes to the ROS topic “/control”
- NOT IMPLEMENTED YET!
### Safety monitor node
- Includes a state machine for controlling tractor speed
    - States:
        - “agopen”
            - No detections have been seen for detection_active_reset_time
            - Forwards agopen control commands directly
        - “moderate”
            - Is entered when a detection is inside safety_distance_1, but not closer
            - Tractor speed is overridden to speed_override_1
            - Transition to state “agopen”, if nothing is detected in detection_active_reset_time
        - “slow”
            - Is entered when a detection is inside safety_distance_2, but not closer
            - Tractor speed is overridden to speed_override_2
            - Transition to state “moderate”, if nothing is detected in detection_active_reset_time
        - “stopped”
            - Is entered when a detection is inside stopping_distance
            - Tractor is stopped
            - Transition to state “slow”, if nothing detected in vehicle_stopped_reset_time
- Subscribes to the ROS topics “/control/agopen” and “/fused_detections”
- Publishes the control commands to the ROS topic “/control”

*Safety monitor state machine:*

![Safety monitor state machine](safety_monitor_state_machine.png)

### Simulation
- Includes nodes and/or python scripts for simulating the hardware components
- Simulations:
    - Target_to_fuse
        - Creates a node publishing matching detections to /camera_detections and /radar_detections simultaneously
        - Used for testing the fusion algorithm
    - Radar_simulator
        - Scripts for simulated UART and CAN connections
        - Publishes random radar detections on /radar_detections
    - Camera_simulator
        - Creates a node publishing random camera detections on /camera_detections
