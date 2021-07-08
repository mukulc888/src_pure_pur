#! /usr/bin/env python3
import math
import numpy as np
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
import message_filters
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import tf2_ros
import tf2_geometry_msgs


target_frame = "base_link"
source_frame = "map"

car_len=0.3302 #meters
look_ahead = 2 #meters
pub = rospy.Publisher('/drive',AckermannDriveStamped , queue_size = 1)
pub2 = rospy.Publisher('/point',PointStamped,queue_size=1)
def get_steering(goal_y):
    steer= math.atan((2*goal_y*car_len)/(look_ahead**2))
    
    if steer > 30*np.pi/180:
        steer = 30*np.pi/180
    if steer < -30*np.pi/180:
        steer = -30*np.pi/180
    return steer 


def get_goal(path,look_dist,x,y):
    a=0
    for i in path:
        if a < look_dist:
            a = math.sqrt((i[0])**2 +(i[1])**2)
            
        else:
            print(a)
            x_goal=i[0]
            y_goal=i[1]
                 
            p_msg = PointStamped()
            p_msg.header.frame_id = 'base_link'
            p_msg.point.x = x_goal 
            p_msg.point.y = y_goal 
            pub2.publish(p_msg)
            break
    return x_goal,y_goal



def transform_pt(in_point):
    tf2_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf2_buffer)

    my_pose = Pose()
    my_pose.position.x = in_point[0]
    my_pose.position.y = in_point[1]
    my_pose.position.z = 0
    my_pose.orientation.x = 0
    my_pose.orientation.y = 0
    my_pose.orientation.z = 0
    my_pose.orientation.w = 1

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = my_pose
    pose_stamped.header.frame_id = source_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
    # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf2_buffer.transform(pose_stamped, target_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

        

def callback(odom,way):
    print('check_call')
    carPosx=odom.pose.pose.position.x
    carPosy=odom.pose.pose.position.y
    print('----------------------')
    print(carPosx,carPosy)
    way_points=[]
    for i in range(len(way.poses)) :
        x=way.poses[i].pose.position.x
        y=way.poses[i].pose.position.y
        way_points.append(transform_pt([x,y]))
    print(way_points)
    """ goal_x,goal_y = get_goal(way_points,look_ahead,carPosx,carPosy)
    steer = get_steering(goal_y)
    print('steer = ',steer*180/np.pi)
    print(goal_x,goal_y)
    print('----------------------')
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = '/map'
    msg.drive.steering_angle = steer
    msg.drive.speed = 1.0 #m/s
    print(msg)
    pub.publish(msg)
 """
if __name__ == "__main__":
    rospy.init_node("path_for_control", anonymous= True)
    
    #  sub_way = rospy.Subscriber('waypoints', WaypointsArray, callback)
    odom_sub = message_filters.Subscriber('/odom', Odometry)
    way_sub = message_filters.Subscriber('/move_base/GlobalPlanner/plan', Path)
    print('check1')
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, way_sub], 1, 0.1, allow_headerless=True)
    print('check2')
    ts.registerCallback(callback)
    print('check3')
    
    # rate = rospy.Rate(1) # small amount on purpose (20m in 20 sec)
    rospy.spin()

""" 
   1 #!/usr/bin/env python
   2 # license removed for brevity
   3 import rospy
   4 from std_msgs.msg import String
   5 
   6 def talker():
   7     pub = rospy.Publisher('chatter', String, queue_size=10)
   8     rospy.init_node('talker', anonymous=True)
   9     rate = rospy.Rate(10) # 10hz
  10     while not rospy.is_shutdown():
  11         hello_str = "hello world %s" % rospy.get_time()
  12         rospy.loginfo(hello_str)
  13         pub.publish(hello_str)
  14         rate.sleep()
  15 
  16 if __name__ == '__main__':
  17     try:
  18         talker()
  19     except rospy.ROSInterruptException:
  20         pass """
  