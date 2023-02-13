#! /usr/bin/env python

## @package rt2_assignment1
# \file go_to_point.py
# \brief Node that implements the go_to_point behavior
# \author Lorenzo Morocutti
# \version 1.0
# \date 1/02/2023
#
# Publishes to:<BR>
#   /cmd_vel(geometry_msgs.msg.Twist)
#   /destination(std_msgs.msg.Float32)
#
# Subscribes to:<BR>
#   /sub_odom(nav_msgs.msg.Odometry)
#   /sub_vel(geometry_msgs.msg.Twist)
#
# ServiceServer:<BR>
#   none
#
# ActionServer:<BR>
#   /go_to_point (rt2_assignment1.action.Moving)
#
# Description:
# 
# The node controls the behavior of the non-holonomic
# according to the go_to_point behavior via an action
# server. A FSM is used to model the behavior every 
# time a new goal pose is received: firstly the robot
# alight its position with the goal's one, then it goes
# straight to the goal position, it aligns with the 
# goal orientation and then the goal pose is reached.
#
# The linear and angular velocities are set by the node
# /user_interface and they are received on the topic 
# /vel. Both the velocities are set to zero in case
# the goal is cancelled by the client.
##

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
from rt2_assignment1 import msg
import time

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


global vel 
vel = Twist()

vel.angular.z = 0.5
vel.linear.x = 0.5

def clbk_odom(msg):
    """!
        This function is the Odometry callback
        It retrieves (x,y,theta) from the Odom message.

        \param msg (Odometry): odometry message.
        \return : None
    """
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


def change_state(state):
    """!
        This function has the scope to update the current global state

        \param state (int): new updated state
        \return : None
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """!
        This function normalizes the angle between [-pi,pi].

        \param angle (float): input angle
        \return angle(float): normalized angle
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def velocity_clbk(velocity):
    """!
        The data received on the topic /vel are saved in vel, a global 
        variable of type Twist.

        \param velocity (float): input velocity
        \return : None
    """
    vel.linear.x = velocity.linear.x
    vel.angular.z = velocity.angular.z

def fix_yaw(des_pos):
    """!
        This function orients the robot in the desidered way.
        The function is used in two situations:
            1. When the robot has to be oriented toward 
               the goal (x,y) position.
            2. When the robot has already reached the goal (x,y) position and 
               it has to achieve the goal orientation.
        It sets the angular velocity to reach the goal position.
        It also updates the state to the behavior go_straight, based on 
        the orientation error.

        \param des_pos ((Point): desidered (x,y) position
        \return : None
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = vel.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """!
        This function implements the behavior that allows the
        robot to reach the goal.
        It sets the linear and angular velocities depending on the
        distance to the goal pose.

        \param des_pos (Point): desidered (x,y) position
        \return : None
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = vel.linear.x * err_pos
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = vel.linear.x

        twist_msg.angular.z = vel.angular.z * err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """!
        This function orients the robot in the desidered 
        orientation, based the error between the current 
        orientation and the desired one.
        It sets the angular velocity to obtain the desidered
        orientation.
        It also updates the state to done, once the orientation
        is set. 

        \param des_yaw (float): desidered (theta) position
        \return : None
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = vel.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """!
        This funciton has the aim of stopping the robot.
        It sets the linear and the angular velocity to zero.

        \param : None
        \return : None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

def time_check(e_t):
    """!
        This function checks the time.

        \param e_t (time): elapsed time
        \return : None
    """
    pub_t.publish(e_t)

def go_to_point(req):
    """!
        This function implements the state machine.
        It sets the correct behavior that the robot
        will follow, based on the current robot state.
        The FMS continues until the goal is reached or
        the goal is cancelled

        \param req (MovingActionGoal):(x,y,theta) goal pose)
        \return : None
    """
    print("request received")
    start_time = time.time()

    desired_position = Point()
    desired_position.x = req.target_pose.pose.position.x
    desired_position.y = req.target_pose.pose.position.y
    des_yaw = req.target_pose.pose.position.z
    change_state(0)
    while True:
        if act.is_preempt_requested():
            rospy.loginfo("Action preempted \n")
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            pub_.publish(velocity)
            act.set_preempted()
            elapsed_time = time.time() - start_time
            time_check(-1)
            break
        elif state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            act.set_succeeded()
            elapsed_time = time.time() - start_time
            time_check(elapsed_time)
            break

    return True

def main():
    global pub_, act, pub_t
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub_t = rospy.Publisher('/destination', Float32, queue_size=1)
    sub_vel = rospy.Subscriber('/vel', Twist, velocity_clbk)
    
    #service = rospy.Service('/go_to_point', Position, go_to_point)
    act = actionlib.SimpleActionServer('/go_to_point', msg.MovingAction, execute_cb=go_to_point, auto_start=False) #(nodehandle, stringname, execute_cb=callback, auto_start)
    act.start()

    rospy.spin()

if __name__ == '__main__':
    main()
