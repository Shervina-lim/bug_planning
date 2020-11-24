#! /usr/bin/env python

# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
# desired_position_.x = rospy.get_param('des_pos_x')
# desired_position_.y = rospy.get_param('des_pos_y')
# desired_position_.z = 0

goal = PoseStamped()

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def goal_callback(goal_data):
    global desired_position_
    desired_position_.x = goal_data.pose.position.x
    desired_position_.y = goal_data.pose.position.y
    desired_position_.z = goal_data.pose.position.z
    if goal_data.pose.position.x == 0 and goal_data.pose.position.y == 0:
        done()
        rospy.signal_shutdown('Stop robot.')

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]', state_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    # Get velocity limits from launch file
    turn_vw = rospy.get_param('max_vw')

    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = turn_vw if err_yaw > 0 else (-1 * turn_vw) # 0.4
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print ('Yaw error: [%s]' ,err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    
    # get velocity from launch file
    go_vw = rospy.get_param('go_vw')
    go_vx = rospy.get_param('go_vx')

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = go_vx # 0.4 in real-life
        twist_msg.angular.z = go_vw if err_yaw > 0 else (-1 * go_vw) # 0.1 in real-life
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]', err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]', err_yaw)
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

# rostopic pub -1 /goal geometry_msgs/Pose '{position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
def main():
    global pub, active_
    global desired_position_

    rospy.init_node('go_to_point')

    # Get topic names from launch file 
    odom = rospy.get_param('odom')
    goal = rospy.get_param('goal')
    cmd_vel = rospy.get_param('cmd_vel')

    pub = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    goal_sub = rospy.Subscriber(goal, PoseStamped, goal_callback)
    odom_sub = rospy.Subscriber(odom, Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rospy.wait_for_message(goal, PoseStamped)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            print("did i crash here?")
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    main()
