#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_arm_command():
    rospy.init_node('arm_goal_publisher', anonymous=True)
    
    # The publisher for the arm controller's command topic
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    rospy.sleep(1.0) 

    # Create the trajectory message
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['j1', 'j2']

    # Create a single point in the trajectory
    point = JointTrajectoryPoint()
    

    # Set the target position for each joint (in radians)
    point.positions = [0.5, 1.0] 
    
    # Set the time to reach this point (e.g., 3 seconds)
    point.time_from_start = rospy.Duration(3.0) 
    # -------------------------

    traj.points = [point]

    # Publish the trajectory
    rospy.loginfo("Sending goal to arm...")
    pub.publish(traj)

if __name__ == '__main__':
    try:
        send_arm_command()
    except rospy.ROSInterruptException:
        pass