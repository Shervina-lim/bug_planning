#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
#initial_position_ = Point()
# initial_position_.x = rospy.get_param('initial_x')
# initial_position_.y = rospy.get_param('initial_y')
# initial_position_.z = 0
desired_position_ = Point()
# desired_position_.x = rospy.get_param('des_pos_x')
# desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
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

def clbk_laser(msg):
    global regions_
    regions_ = {
        # 'right':  min(min(msg.ranges[0:143]), 10),
        # 'fright': min(min(msg.ranges[144:287]), 10),
        # 'front':  min(min(msg.ranges[288:431]), 10),
        # 'fleft':  min(min(msg.ranges[432:575]), 10),
        # 'left':   min(min(msg.ranges[576:719]), 10),
        #'f-right2':   min(min(msg.ranges[0:108]), 50), # 50 is the maximum value we can read
        #'f-right1':   min(min(msg.ranges[108:216]), 50), # 50 because LiDAR specs is 50m. 
        #'front':      min(min(msg.ranges[216:324]), 50),
        #'f-left1':    min(min(msg.ranges[324:432]), 50),
        #'f-left2':    min(min(msg.ranges[432:540]), 50),
        #'b-left2':    min(min(msg.ranges[540:648]), 50),
        #'b-left1':    min(min(msg.ranges[648:756]), 50),
        #'back':       min(min(msg.ranges[756:864]), 50),
        #'b-right1':   min(min(msg.ranges[864:972]), 50),
        #'b-right2':    min(min(msg.ranges[972:1083]), 50)
        
        #'front':   min(min(msg.ranges[0:54] + msg.ranges[1026:1083]), 30), # 50 is the maximum value we can read
        #'f-left1':   min(min(msg.ranges[54:162]), 30), # 50 because LiDAR specs is 50m. 
        #'f-left2':      min(min(msg.ranges[162:270]), 30),
        #'b-left2':    min(min(msg.ranges[270:378]), 30),
        #'b-left1':    min(min(msg.ranges[378:486]), 30),
        #'back':    min(min(msg.ranges[486:594]), 30),
        #'b-right1':    min(min(msg.ranges[594:702]), 30),
        #'b-right2':       min(min(msg.ranges[702:810]), 30),
        #'f-right2':   min(min(msg.ranges[810:918]), 30),
        #'f-right1':    min(min(msg.ranges[918:1026]), 30)

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

def goal_callback(goal_data):
    global desired_position_
    desired_position_.x = goal_data.pose.position.x
    desired_position_.y = goal_data.pose.position.y
    desired_position_.z = goal_data.pose.position.z
    rospy.loginfo('Goal message received.')

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    print("did i crash here?")
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0: # go to point
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1: # wall following
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # print(p2)
    # here goes the equation
    #print('odom position: ', p0.x, ' and ', p0.y)
    #print('goal position: ', p2.x, ' and ', p2.y)
    #up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    #lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = math.fabs(math.sqrt(pow(p2.y - p0.y, 2) + pow(p2.x - p0.x, 2)))
    #try:
        #distance = up_eq / lo_eq
    #    distance = math.sqrt(pow(p2.y - p0,y, 2) + pow(p2.x - p0.x, 2))
    #except:
    #    distance = 0

    return distance
    

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# rostopic pub -1 /goal geometry_msgs/Pose '{position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_
    
    rospy.init_node('bug0')
    
    # Get topic names from launch file 
    scan = rospy.get_param('scan')
    odom = rospy.get_param('odom')
    goal = rospy.get_param('goal')
    cmd_vel = rospy.get_param('cmd_vel')

    sub_laser = rospy.Subscriber(scan, LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber(odom , Odometry, clbk_odom)
    goal_sub = rospy.Subscriber(goal, PoseStamped, goal_callback)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    goal_pub = rospy.Publisher(goal, PoseStamped, queue_size=1)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    # rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # For gazebo, not required since we use pybullet
    # set robot position 
   # model_state = ModelState()
   # model_state.model_name = 'Omnirobot'
   # model_state.pose.position.x = initial_position_.x
    #model_state.pose.position.y = initial_position_.y
    # resp = srv_client_set_model_state(model_state)
    
    print('Waiting for goal message...')
    rospy.wait_for_message(goal, PoseStamped)
    # initialize going to the point
    change_state(0)
    
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        distance_position_to_line = distance_to_line(position_)
        
        if state_ == 0: # when go to point
            if regions_['front'] > 0.50 and regions_['front'] < 1:
                change_state(1) # become wall-following
        
        elif state_ == 1:
            if count_state_time_ > 5 and \
               distance_position_to_line < 0.1:
                change_state(0)
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        if distance_to_line(position_) <= 0.30: # reached goal
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg) # publish zero velocity
# is it really necessary to clear goal?
            stop_msg = PoseStamped()
            stop_msg.pose.position.x = 0.0
            stop_msg.pose.position.y = 0.0
            stop_msg.pose.position.z = 0.0
            goal_pub.publish(stop_msg)
            rospy.signal_shutdown('Reached goal.')
        #print('36 degree forward', regions_['front'])
        #print('36 degree top left', regions_['f-left1'])
        #print('36 degree above horizontal left', regions_['f-left2'])
        #print('36 degree below horizontal left', regions_['b-left2'])
        #print('36 degree bottom left', regions_['b-left1'])
        #print('36 degree backwards', regions_['back'])
        #print('36 degree bottom right', regions_['b-right1'])
        #print('36 degree below horizontal right', regions_['b-right2'])
        #print('36 degree above horizontal right', regions_['f-right2'])
        #print('36 degree top right', regions_['f-right1'])
        rate.sleep()

if __name__ == "__main__":
    main()
