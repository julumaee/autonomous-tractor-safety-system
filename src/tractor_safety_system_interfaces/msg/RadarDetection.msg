# This is a message type defined for radar detections.
# Detections published from the radar node will be of this type.

std_msgs/Header header
geometry_msgs/Point position

float32 distance  # Distance in meters
float32 speed     # Speed in meters per second

# Radar-reported confidence/probability for this detection.
# 0..100 (%). Can be used to gate single-sensor publishes while still allowing fusion.
float32 confidence_percent
