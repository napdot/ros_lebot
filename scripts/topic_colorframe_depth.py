import rospy
from m




image_sub = message_filters.Subscriber("image", Image)
depth_sub = message_filters.Subscriber("depth_image", Image)
sync = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)
sync.registerCallback(mask_detect_callback)