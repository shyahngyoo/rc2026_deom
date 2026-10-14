# A producer binds each command to the current authoritative lease generation.
std_msgs/Header header
string owner
string mission_id
uint64 generation
geometry_msgs/Twist twist
