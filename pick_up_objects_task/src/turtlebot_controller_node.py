#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """ Computes the control command to move from current position to goal. """
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.05: # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w

class Controller:
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):

        # Attributes
        self.distance_threshold = distance_threshold                                                # Distance threshold to way point
        self.current_pose = None                                                                    # Current robot SE2 pose                           
        self.goal = None                                                                            # A goal is set
        self.path = []                                                                              # List of points which define the plan. None if there is no plan
        # Parameters
        self.Kv = 0.5                   # Proportional linear velocity controller
        self.Kw = 0.5                   # Proportional angular velocity controller
        self.v_max = 0.15               # Maximum linear velocity control action
        self.w_max = 0.3                # Maximum angular velocity control action

        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2], self.goal]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0   
        w = 0
        if self.path is not None and len(self.path) > 0:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)



if __name__ == '__main__':
    rospy.init_node('turtlebot_controller')   
    node = Controller('/odom', '/cmd_vel', 0.15)
    
    # Run forever
    rospy.spin()
