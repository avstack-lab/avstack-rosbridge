# A representation of a box track with uncertainty
# assumed state vector is [x, y, w, h, vx, vy]

std_msgs/Header header

string obj_type

vision_msgs/BoundingBox2D box

float64 vel_x

float64 vel_y

float64[36] p  # state vector covariance

uint32 n_updates 0

float64 dt_coast 0.0

uint32 identifier 0

float64 score 0.0