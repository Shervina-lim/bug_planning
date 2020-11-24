#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 1080 / 10 = 108
    regions = [
        min(min(msg.ranges[0:108]), 50), # 50 is the maximum value we can read
        min(min(msg.ranges[108:216]), 50), # 50 because LiDAR specs is 50m. 
        min(min(msg.ranges[216:324]), 50),
        min(min(msg.ranges[324:432]), 50),
        min(min(msg.ranges[432:540]), 50),
        min(min(msg.ranges[540:648]), 50),
        min(min(msg.ranges[648:756]), 50),
        min(min(msg.ranges[756:864]), 50),
        min(min(msg.ranges[864:972]), 50),
        min(min(msg.ranges[972:1083]), 50),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    
    # Get topic names from launch file 
    scan = rospy.get_param('scan')

    sub = rospy.Subscriber(scan, LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
