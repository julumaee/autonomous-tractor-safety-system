# This is a message type defined for camera detections.
# Detections published from the camera node will be of this type.

std_msgs/Header header

# Class probabilities
vision_msgs/ObjectHypothesis[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# Center of the detected object in meters 
geometry_msgs/Point position
