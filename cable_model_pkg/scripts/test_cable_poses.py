#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose
from cable_model_pkg.srv import GraspMsg

def test_cable_poses():
    rospy.init_node('cable_test_publisher', anonymous=True)
    
    # Publishers for robot poses
    mass0_pub = rospy.Publisher('/mass0/pose', Pose, queue_size=10)
    massN_pub = rospy.Publisher('/massN/pose', Pose, queue_size=10)
    
    # Wait for grasp service
    rospy.wait_for_service('set_cable_grasp')
    grasp_service = rospy.ServiceProxy('set_cable_grasp', GraspMsg)
    
    rate = rospy.Rate(50)  # 50 Hz
    
    rospy.loginfo("Starting cable pose test...")
    
    # Wait a bit for everything to initialize
    rospy.sleep(2.0)
    
    # Enable grasping on both ends
    try:
        response = grasp_service(grasp_mass_0=True, grasp_mass_N=True)
        rospy.loginfo("Cable grasp enabled: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return
    
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed = (current_time - start_time).to_sec()
        
        # Create oscillating motion for mass 0 (left end)
        pose_mass0 = Pose()
        pose_mass0.position.x = -0.5 + 0.2 * math.sin(0.5 * elapsed)
        pose_mass0.position.y = 0.0
        pose_mass0.position.z = 1.2 + 0.1 * math.cos(0.5 * elapsed)
        pose_mass0.orientation.w = 1.0
        pose_mass0.orientation.x = 0.0
        pose_mass0.orientation.y = 0.0
        pose_mass0.orientation.z = 0.0
        
        # Create oscillating motion for mass N (right end)
        pose_massN = Pose()
        pose_massN.position.x = 0.5 + 0.2 * math.sin(0.3 * elapsed + math.pi/4)
        pose_massN.position.y = 0.0
        pose_massN.position.z = 1.2 + 0.1 * math.cos(0.3 * elapsed + math.pi/4)
        pose_massN.orientation.w = 1.0
        pose_massN.orientation.x = 0.0
        pose_massN.orientation.y = 0.0
        pose_massN.orientation.z = 0.0
        
        # Publish poses
        mass0_pub.publish(pose_mass0)
        massN_pub.publish(pose_massN)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        test_cable_poses()
    except rospy.ROSInterruptException:
        pass
