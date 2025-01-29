# This is a message type defined for camera detections.
# Detections published from the camera node should be of this type.

std_msgs/Header header
vision_msgs/ObjectHypothesis[] results  # Class probabilities
vision_msgs/BoundingBox2D bbox          # 2D bounding box surrounding the object
geometry_msgs/Point position            # Center of the detected object in meters

int32 distance                          # Distance in meters
int32 speed                             # Speed in meters per second

bool is_tracking # If true, this message contains object tracking information.

# ID used for consistency across multiple detection messages. This value will
# likely differ from the id field set in each individual ObjectHypothesis.
# If you set this field, be sure to also set is_tracking to True.
string tracking_id