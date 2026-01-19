# This is a message type defined for fused detections.
# Detections published from the fusion node should be
# of this type.

std_msgs/Header header
vision_msgs/ObjectHypothesis[] results  # Class probabilities
geometry_msgs/Point position            # Center of the detected object in meters

float32 distance    # Distance in meters
float32 speed       # Speed in meters per second

bool is_tracking  # If true, this message contains object tracking information.

# ID used for consistency across multiple detection messages. This value will
# likely differ from the id field set in each individual ObjectHypothesis.
# If you set this field, be sure to also set is_tracking to True.
string tracking_id

# Type of detection. Used to differentiate
# between fused, radar, and camera detections.
# Valid values are "fused", "radar", "camera" and "tracked".
string detection_type

# Values for tracked objects. Used only if if_tracking is set to True.
uint32 age                    # Number of update cycles
uint32 consecutive_misses     # How many updates missed