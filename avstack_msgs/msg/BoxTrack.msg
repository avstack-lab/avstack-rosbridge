# A representation of a box track with uncertainty
# assumed state vector is [x, y, z, h, w, l, vx, vy, vz] with attitude as an attribute

string obj_type

vision_msgs/BoundingBox3D box

geometry_msgs/Vector3 velocity

float64[81] p  # state vector covariance

uint32 n_updates 0

float64 dt_coast 0.0

uint32 identifier 0