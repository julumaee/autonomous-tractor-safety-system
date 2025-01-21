# This is a message type defined for radar detections.
# Detections published from the radar node will be of this type.

std_msgs/Header header

int32 distance  # Distance in meters
int32 angle     # Angle in degrees
int32 speed     # Speed in meters per second