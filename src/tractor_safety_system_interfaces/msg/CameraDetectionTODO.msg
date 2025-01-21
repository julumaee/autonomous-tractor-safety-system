# This is a message type defined for camera detections.
# Detections published from the camera node should be of this type.

std_msgs/Header header
string label                    # Detected object label (e.g., "person", "car")
float32 confidence              # Confidence score of the detection
float32 depth                   # Depth to the detected object in meters
geometry_msgs/Point position    # Object's position in 3D space
geometry_msgs/Vector3 size      # Size of the detected object (width, height, depth)