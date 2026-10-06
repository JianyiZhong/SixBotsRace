# Target msg for object detection
#

std_msgs/Header header

int32 frame_width
int32 frame_height
int32 frame_index
int32 track_state

#tracker: 0, 1, 2 indicate tracking, gt_tracking, lost
#GroundController: 0, select target; 1, reset; 2, fixpoint; 3, follow; 4, hit;
#box x, y, w, h
controller_msgs/TargetBox[] bbox

