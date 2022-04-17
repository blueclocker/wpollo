#this is the gnss message,include lla, rpy, velocity

#The header.
Header header

# satellite fix status information
int8 status
uint8 PositionOK
uint8 HeadingOK
uint8 WorkMode
uint8 NoSV

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# The rpy[du] of this pose.
float64 roll
float64 pitch
float64 yaw

# The velocity[m/s] in ENU coordirate.
float64 vel_e
float64 vel_n
float64 vel_a


