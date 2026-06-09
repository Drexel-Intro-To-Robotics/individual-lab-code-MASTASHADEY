#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

def send_goal():
    rospy.init_node('test_goal_publisher', anonymous=True)
    
    # Connect directly to the topic your Task 8 node is listening to
    pub = rospy.Publisher('/goal_task_space', Point, queue_size=10)
    
    # Wait 1 second to ensure the network connection is fully established
    rospy.sleep(1.0) 
    
    # Package the safe coordinates
    msg = Point()
    msg.x = 0.15
    msg.y = 0.0
    msg.z = 0.10
    
    rospy.loginfo(f"Sending coordinates: X={msg.x}, Y={msg.y}, Z={msg.z}")
    pub.publish(msg)
    
    # Wait 1 second before shutting down so the message isn't dropped
    rospy.sleep(1.0)
    rospy.loginfo("Message sent!")

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass
