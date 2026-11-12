std_msgs/Header header
string elevator_id
string state            # IDLE | DOORS_OPEN | DOORS_OPENING | DOORS_CLOSING | MOVING | FAULT | EMERGENCY
int32 current_floor     # nearest floor (1-based), -1 while between floors
int32 target_floor      # -1 when none
float64 height
float64 door_opening    # 0..1 for the doors at current_floor
bool doorway_blocked
string owner            # requester of the active goal ('' if none)
string fault            # last fault description ('' if none)
