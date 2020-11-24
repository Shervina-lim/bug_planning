#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None
regions_ = {
        'f-right2': 0,
        'f-right1': 0,
        'front': 0,
        'f-left1': 0,
        'f-left2': 0,
        'b-left2': 0,
        'b-left1': 0,
        'back': 0,
        'b-right1': 0,
        'b-right2': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    regions_ = { # Requires verification that LiDAR data 
        'back':   min(min(msg.ranges[0:54] + msg.ranges[1026:1083]), 30), # 50 is the maximum value we can read
        'b-right1':   min(min(msg.ranges[54:162]), 30), # 50 because LiDAR specs is 50m. 
        'b-right2':      min(min(msg.ranges[162:270]), 30),
        'f-right2':    min(min(msg.ranges[270:378]), 30),
        'f-right1':    min(min(msg.ranges[378:486]), 30),
        'front':    min(min(msg.ranges[486:594]), 30),
        'f-left1':    min(min(msg.ranges[594:702]), 30),
        'f-left2':       min(min(msg.ranges[702:810]), 30),
        'b-left2':   min(min(msg.ranges[810:918]), 30),
        'b-left1':    min(min(msg.ranges[918:1026]), 30)
    }
    
    take_action()

def clbk_goal(msg):
    if msg.pose.position.x == 0 and msg.pose.position.y == 0:
        rospy.signal_shutdown('Robot reached goal, closing follow_wall node.')

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s') % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    #print('36 degree from horizontal datum', regions['f-right2'])
    #print('36 degree top right', regions['f-right1'])
    #print('36 degree forward', regions['front'])
    #print('36 degree top left', regions['f-left1'])
    #print('36 degree from neg horizontal datum', regions['f-left2'])
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.0 # Default is 1.5 m threshold. Robot radius is 30 cm. # distance to wall from robot
    # env 5 and 6: 0.8
    #if regions['front'] > d*2 and regions['f-left1'] > d*2 and regions['f-right1'] > d*2:
    #    state_description = 'case 1 - nothing'
    #    change_state(0) # become find the wall, keep turning right and going forward
    # Edited the following condition to do follow wall    
    if regions['front'] > d and regions['f-left1'] > d and regions['f-right1'] > d:
        state_description = 'case 1 - nothing'
        change_state(0) # become find the wall, keep turning right and going forward
    elif regions['front'] < d and regions['f-left1'] > d and regions['f-right1'] > d:
        state_description = 'case 2 - front'
        change_state(1) # turn left
    # The only situation where the robot will begin to follow wall is this, when there is nothing in the front-left and front-right view of robot. 
    elif regions['front'] > d and regions['f-left1'] > d and regions['f-right1'] < d:
        state_description = 'case 3 - fright1'
        change_state(2) # follow the wall
    elif regions['front'] > d and regions['f-left1'] < d and regions['f-right1'] > d:
        state_description = 'case 4 - fleft1'
        # is finding wall the correct command to do here? 
        change_state(0) # become find the wall
        # change to follow wall
        # change_state(2)

    elif regions['front'] < d and regions['f-left1'] > d and regions['f-right1'] < d:
        state_description = 'case 5 - front and fright1'
        change_state(1) # turn left
    elif regions['front'] < d and regions['f-left1'] < d and regions['f-right1'] > d:
        state_description = 'case 6 - front and fleft1'
        change_state(1) # turn left
    elif regions['front'] < d and regions['f-left1'] < d and regions['f-right1'] < d:
        state_description = 'case 7 - front and fleft1 and fright1'
        change_state(1) # turn left
    # This could be what is causing the robot to keep turning left because the corridor is too tight
    elif regions['front'] > d and regions['f-left1'] < d and regions['f-right1'] < d:
        state_description = 'case 8 - fleft1 and fright1'
        change_state(0) # become find the wall
        # Well this algorithm will definitely fail when goals are backwards of the robot due to this condition. 
        # This hardcoded algorithm will not react well to dynamic obstacles. 
        # change_state(2)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    # original code
    '''if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0) # become find the wall
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2) # follow the wall
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)'''

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.5
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5 # 0.5 # 1.5
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
        # Get topic names from launch file 
    scan = rospy.get_param('scan')
    goal = rospy.get_param('goal')
    cmd_vel = rospy.get_param('cmd_vel')

    pub_ = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    
    sub = rospy.Subscriber(scan, LaserScan, clbk_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    sub_cmd_vel = rospy.Subscriber(goal, PoseStamped, clbk_goal)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        print("here?")
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
