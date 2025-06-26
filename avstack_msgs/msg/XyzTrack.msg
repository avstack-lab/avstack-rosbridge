# A representation of an xyz track with uncertainty
# assumed state vector is [x, y, z, vx, vy, vz] with attitude as an attribute

std_msgs/Header header

string obj_type

geometry_msgs/Vector3 position

geometry_msgs/Vector3 velocity

geometry_msgs/Vector3 attitude

float64[36] p  # state vector covariance

uint32 n_updates 0

float64 dt_coast 0.0

uint32 identifier 0

float64 score 0.0