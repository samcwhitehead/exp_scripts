  ### for viewing debugging bag files

   1 import rosbag
   2 bag = rosbag.Bag('ledpanels_debug.bag')
   3 for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
   4     print msg
   5 bag.close()