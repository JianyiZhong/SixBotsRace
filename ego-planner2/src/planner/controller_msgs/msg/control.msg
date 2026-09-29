# reference 
# cmd: 0, takeoff mode(offboard); 
#      1, hover mode(posctl)
#      2. execution mode(offboard); meantime, publish waypoints
#      3, landing
#      4, execution mode(offboard); send goal for test

std_msgs/Header header

uint8 cmd
int16 drone_id
