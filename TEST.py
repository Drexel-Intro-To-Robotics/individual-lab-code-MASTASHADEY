#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_raw_command():
    rospy.init_node('raw_arm_publisher', anonymous=True)
    
    # 1. Connect directly to the raw controller topic
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    # Wait 1 second to ensure the node registers on the ROS network
    rospy.sleep(1.0)
    
    # 2. Build the main trajectory envelope
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    
    # Setting time to 0 forces the controller to execute immediately 
    # and prevents clock synchronization crashes between nodes
    traj.header.stamp = rospy.Time(0) 
    
    # 3. Build the specific waypoint you want to reach
    point = JointTrajectoryPoint()
    point.positions = [0.0, -0.2, 0.2, 0.0]
    
    # Leave these explicitly empty to prevent C++ array bounds errors
    point.velocities = []
    point.accelerations = []
    point.time_from_start = rospy.Duration(3.0)
    
    # Attach the waypoint to the trajectory envelope
    traj.points.append(point)
    
    # 4. Fire the message
    rospy.loginfo("Publishing raw JointTrajectory to the arm...")
    pub.publish(traj)
    
    # Wait 1 second before the script ends to guarantee network delivery
    rospy.sleep(1.0)
    rospy.loginfo("Command successfully sent!")

if __name__ == '__main__':
    try:
        send_raw_command()
    except rospy.ROSInterruptException:
        pass
