# Message header
Header header

# Map header encapsulating the map's main properties
cblox_msgs/MapHeader map_header

# Layer type
# 0 - TSDF
# 1 - ESDF
uint8 type

# Layer containing the map's TSDF or ESDF
# NOTE: It is expressed in the frame specified by header.frame_id
voxblox_msgs/Layer tsdf_layer
voxblox_msgs/Layer esdf_layer
