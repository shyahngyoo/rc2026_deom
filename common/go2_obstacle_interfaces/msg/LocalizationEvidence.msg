# Atomically associates a match verdict and measurement pose; rejected events never
# refresh accepted evidence. source_id identifies one process instance; epoch
# increments on reinitialization/time reset; sequence increases for every event.
std_msgs/Header header
string source_id
uint64 epoch
uint64 sequence
bool accepted
bool covariance_known
geometry_msgs/PoseWithCovariance pose
string body_frame
string reason
