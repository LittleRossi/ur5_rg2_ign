#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float64
import time

def pick_ball():
    # Initialisiere den Node
    rospy.init_node('pick_ball', anonymous=True)
    
    # Initialisiere MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Erstelle einen Roboter-Commander
    robot = moveit_commander.RobotCommander()
    
    # Erstelle einen Szenen-Commander
    scene = moveit_commander.PlanningSceneInterface()
    
    # Erstelle einen Bewegungsgruppen-Commander für den Arm
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    
    # Setze die Planungszeit
    arm_group.set_planning_time(5)
    
    # Erstelle Publisher für die Greifer-Gelenke
    pub_finger1 = rospy.Publisher('/rg2_finger_joint1_position_controller/command', Float64, queue_size=10)
    pub_finger2 = rospy.Publisher('/rg2_finger_joint2_position_controller/command', Float64, queue_size=10)
    
    # Warte, bis die Publisher verbunden sind
    rospy.sleep(2)
    
    # Öffne den Greifer
    rospy.loginfo("Öffne den Greifer")
    for i in range(11):
        position = i * 0.1
        pub_finger1.publish(Float64(position))
        pub_finger2.publish(Float64(position))
        rospy.sleep(0.1)
    rospy.sleep(1)
    
    # Bewege den Arm zur Startposition
    rospy.loginfo("Bewege zur Startposition")
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    
    arm_group.go(joint_goal, wait=True)
    arm_group.stop()
    
    # Bewege den Arm über den Ball
    rospy.loginfo("Bewege über den Ball")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = -0.7071068
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0.7071068
    pose_goal.position.x = 0.7
    pose_goal.position.y = 0
    pose_goal.position.z = 0.9  # Höhe über dem Tisch + Ball
    
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    # Bewege den Arm zum Ball
    rospy.loginfo("Bewege zum Ball")
    waypoints = []
    wpose = arm_group.get_current_pose().pose
    wpose.position.z -= 0.1
    waypoints.append(wpose)
    
    (plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # Wegpunkte
                                   0.01,        # Auflösung
                                   0.0)         # Sprungdistanz
    
    arm_group.execute(plan, wait=True)
    
    # Schließe den Greifer
    rospy.loginfo("Schließe den Greifer")
    for i in range(11):
        position = 1.0 - (i * 0.1)
        pub_finger1.publish(Float64(position))
        pub_finger2.publish(Float64(position))
        rospy.sleep(0.1)
    rospy.sleep(1)
    
    # Hebe den Ball auf
    rospy.loginfo("Hebe den Ball auf")
    waypoints = []
    wpose = arm_group.get_current_pose().pose
    wpose.position.z += 0.2
    waypoints.append(wpose)
    
    (plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # Wegpunkte
                                   0.01,        # Auflösung
                                   0.0)         # Sprungdistanz
    
    arm_group.execute(plan, wait=True)
    
    # Bewege den Arm zur Zielposition
    rospy.loginfo("Bewege zur Zielposition")
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = pi/2
    
    arm_group.go(joint_goal, wait=True)
    arm_group.stop()
    
    # Öffne den Greifer, um den Ball fallen zu lassen
    rospy.loginfo("Lasse den Ball fallen")
    for i in range(11):
        position = i * 0.1
        pub_finger1.publish(Float64(position))
        pub_finger2.publish(Float64(position))
        rospy.sleep(0.1)
    
    # Zurück zur Startposition
    rospy.loginfo("Zurück zur Startposition")
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    
    arm_group.go(joint_goal, wait=True)
    arm_group.stop()
    
    # Beende MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        pick_ball()
    except rospy.ROSInterruptException:
        pass 