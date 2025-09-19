# This is a message type defined for radar detections.
# Detections published from the radar node will be of this type.

std_msgs/Header header
geometry_msgs/Point position

float32 distance  # Distance in meters
float32 speed     # Speed in meters per second
