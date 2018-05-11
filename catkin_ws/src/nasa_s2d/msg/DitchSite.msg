# A potential ditch site where the vehicle could land in an emergency

# The name of the ditch site
string name

# LLA of the ditch site
geographic_msgs/GeoPoint position

# ditch sites are defined as circles with radius R
float64 radius

# Type of ditch site (?)
int32 type

# How reliable is this site (prioritization)
float64 reliability