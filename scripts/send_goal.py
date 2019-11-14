#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
import numpy as np
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from nav_msgs.srv import GetPlan
from std_msgs.msg import String

class GotoPoint():
   def __init__(self):
      client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
      client.wait_for_server()
      self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
      position = Point()
      move_cmd = Twist()
      r = rospy.Rate(10)
      self.tf_listener = tf.TransformListener()
      self.odom_frame = 'odom'
      
      try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
      except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

 
      (position_init, rotation) = self.get_odom()

      (goal_x, goal_y, goal_z) = x, y, z

      self.start = PoseStamped()
      #self.start.header.seq = 0 
      self.start.header.frame_id = "map"
      self.start.header.stamp = rospy.Time(0)
      self.start.pose.position.x = position_init.x
      self.start.pose.position.y = position_init.y

      self.goal = PoseStamped()
      #self.goal.header.seq = 0
      self.goal.header.frame_id = "map"
      self.goal.header.stamp = rospy.Time(0)
      self.goal.pose.position.x = goal_x
      self.goal.pose.position.y = goal_y

      self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
      self.req = GetPlan()
      self.req.start = self.start
      self.req.goal = self.goal
      self.req.tolerance = .5
      self.resp = self.get_plan(self.req.start, self.req.goal, self.req.tolerance)

      print("plan : %f", self.resp)


      if goal_z > 180 or goal_z < -180:
         print("you input wrong z range.")
         #shutdown()
      goal_z = np.deg2rad(goal_z)

      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.position.x = goal_x
      goal.target_pose.pose.position.y = goal_y
      goal.target_pose.pose.orientation.w = 1.0

      client.send_goal(goal)
      wait = client.wait_for_result()
      if not wait:
         rospy.logerr("Action server not available")
         rospy.signal_shutdown("Action server not available")
      else:
         result = client.get_result()
         if result:
            (position, rotation) = self.get_odom()
            rospy.loginfo("rotation : %f", position.x)
            rospy.loginfo("goal_z : %f", goal_z)

            while abs(rotation - goal_z) > 0.05:
               (position, rotation) = self.get_odom()
               if goal_z >= 0:
                  if rotation <= goal_z and rotation >= goal_z - pi:
                     move_cmd.linear.x = 0.00
                     move_cmd.angular.z = 0.5
                  else:
                     move_cmd.linear.x = 0.00
                     move_cmd.angular.z = -0.5
               else:
                  if rotation <= goal_z + pi and rotation > goal_z:
                     move_cmd.linear.x = 0.00
                     move_cmd.angular.z = -0.5
                  else:
                     move_cmd.linear.x = 0.00
                     move_cmd.angular.z = 0.5
               self.cmd_vel.publish(move_cmd)
               r.sleep()

            rospy.loginfo("Stopping the robot...")
            self.cmd_vel.publish(Twist())


   def get_odom(self):
      try:
         (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
         rotation = euler_from_quaternion(rot)

      except (tf.Exception, tf.ConnectivityException, tf.LookupException):
         rospy.loginfo("TF Exception")
         return

      return (Point(*trans), rotation[2])


def callback(data):
    rospy.loginfo(data.data)
    global x, y, z
    x, y, z = data.data.split('/')
    x, y, z = [float(x), float(y), float(z)]

    pub = rospy.Publisher('complete', String, queue_size=30)

    try:
      while not rospy.is_shutdown():
         GotoPoint()
         pub.publish('1')
         rospy.loginfo("Goal execution done")
    except rospy.ROSInterruptException:
      rospy.loginfo("Navigation test finished")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('send_goal_py')
    rospy.Subscriber("navimode", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
   listener()
