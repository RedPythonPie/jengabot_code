#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
import tf2_ros
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from path_planner import PathPlanner
import os



def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    
    # TODO: initialize a tf buffer and listener as in lab 3
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(3)
    try:
        # TODO: lookup the transform and save it in trans
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', 'ar_marker_' + str(tag_number), rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Set ref frame
    header = Header(stamp=rospy.Time.now(), frame_id='ar_marker_15') # MAKE SURE AR MARKER IS SET PROPERLY EACH TIME

    # Create a PoseStamped and specify header
    pose_stamped = PoseStamped()
    pose_stamped.header = header
    pose_stamped.pose.position = Point(0.24, 0, 0.11) # TODO: ADD POS, would be nice if we can have user input how tall the tower is, that way we can dynamically set the center of mass
    pose_stamped.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)
    tower_size = np.array((0.1425, 0.1425, 0.29)) # (0.15, 0.15, 0.33)
    path_planner = PathPlanner('right_arm')
    path_planner.add_box_obstacle(tower_size, 'Jenga Tower', pose_stamped)

    # Create a PoseStamped and specify header
    # header_table = Header(stamp=rospy.Time.now(), frame_id='ar_marker_15')
    # pose_stamped_table = PoseStamped()
    # pose_stamped_table.header = header_table
    # pose_stamped_table.pose.position = Point(0.25, 0, -0.1) # TODO: ADD POS, would be nice if we can have user input how tall the tower is, that way we can dynamically set the center of mass
    # pose_stamped_table.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)
    # table_size = np.array((1, 0.3, 0.03)) # (0.15, 0.15, 0.33)
    # path_planner_table = PathPlanner('right_arm')
    # path_planner_table.add_box_obstacle(table_size, 'Jenga Tower', pose_stamped_table)

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Get AR Tag Positions (Update these as necessary, or make user input)
    target_pos = lookup_tag(15)
    target_pos[2] -= 0.085
    stick_pos = lookup_tag(9)
    og_stick_pos = stick_pos.copy()
    stick_pos[0] -= 0.16
    stick_pos[2] -= 0.055
    const_tower_offset = 0

    right_gripper.open()
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        tuck_cmd = "roslaunch intera_examples sawyer_tuck.launch"
        os.system(tuck_cmd)

        requested_row = int(input('Which row would you like to try? (Enter a number, with -1 as a stopping condition) \n'))
        if requested_row == -1:
            break
        requested_column = int(input('Which block in the row would you like to try? (1 being closest to you) \n'))

        block_height = 0.03

        height_offset = block_height * requested_row - block_height / 2.0
        side_offset = 0.0
        if requested_column == 1:
            side_offset += 0.04
        elif requested_column == 3:
            side_offset -= 0.05

        after_placement = stick_pos.copy()
        after_placement[2] += 0.04

        aim_pos = stick_pos.copy()
        aim_pos[2] = target_pos[2] + height_offset + 0.005
        aim_pos[0] = target_pos[0] + side_offset + 0.025

        push_pos = aim_pos.copy()
        push_pos[1] = aim_pos[1] - 0.235 + 0.035 # - 0.02

        grab_pos = aim_pos.copy()
        grab_pos[2] -= 0.01 # + 0.025 # For sideways grab
        grab_pos[1] = target_pos[1] + 0.06 + 0.035
        grab_pos[0] += 0.005 # MAY NOT NEED IN FUTURE ITERATIONS

        pre_grab_pos = grab_pos.copy()
        pre_grab_pos[2] += 0.13
        pre_grab_pos[1] -= 0.01

        after_drop_stick = after_placement.copy()
        after_drop_stick[2] = pre_grab_pos[2]

        pull_pos = grab_pos.copy()
        pull_pos[1] -= 0.2

        drop_pos = target_pos.copy()
        drop_pos[0] += const_tower_offset
        drop_pos[1] -= 0.2
        drop_pos[2] += 0.05

        pre_tuck = drop_pos.copy()
        pre_tuck[2] += 0.15

        move_list = [
            # og_stick_pos, # TEMP
            after_placement,
            stick_pos, # Grab stick 
            aim_pos, # Pick up stick
            push_pos, # Push block
            aim_pos, # Pull stick back
            stick_pos, # Set stick down
            after_placement, # Lift up a bit
            after_drop_stick, # Move up
            pre_grab_pos, # Prepare to grab
            grab_pos, # Grab block
            pull_pos, # Pull out block
            drop_pos, # Put block down
            pre_tuck
        ]

        for i in range(len(move_list)):
            
            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"

            # if i == 0:
            #     offset = 0
            #     temp_cont_2 = input('Continue Testing? [y/n]\n')
            #     while temp_cont_2 == 'y':
            #         request.ik_request.pose_stamped.pose.position.x = grab_pos[0] + offset
            #         request.ik_request.pose_stamped.pose.position.y = grab_pos[1]
            #         request.ik_request.pose_stamped.pose.position.z = grab_pos[2]
            #         request.ik_request.pose_stamped.pose.orientation.x = np.sqrt(2) / 2
            #         request.ik_request.pose_stamped.pose.orientation.y = - np.sqrt(2) / 2
            #         request.ik_request.pose_stamped.pose.orientation.z = 0
            #         request.ik_request.pose_stamped.pose.orientation.w = 0
                    
            #         try:
            #             # Send the request to the service
            #             response = compute_ik(request)
                        
            #             # Print the response HERE
            #             print(response)
            #             group = MoveGroupCommander("right_arm")
            #             group.set_max_velocity_scaling_factor(0.3)

            #             # Setting position and orientation target
            #             group.set_pose_target(request.ik_request.pose_stamped)

            #             # TRY THIS
            #             # Setting just the position without specifying the orientation
            #             # group.set_position_target([0.5, 0.5, 0.0])

            #             # Plan IK
            #             plan = group.plan()
            #             print("Offset Performed: " + str(offset))
            #             offset -= 0.005
            #             user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                        
            #             # Execute IK if safe
            #             if user_input == 'y':
            #                 group.execute(plan[1])
                        
            #         except rospy.ServiceException as e:
            #             print("Service call failed: %s"%e)

            #         temp_cont_2 = input('Continue Testing? [y/n]\n')


            request.ik_request.pose_stamped.pose.position.x = move_list[i][0]
            request.ik_request.pose_stamped.pose.position.y = move_list[i][1]
            request.ik_request.pose_stamped.pose.position.z = move_list[i][2]  
            request.ik_request.pose_stamped.pose.orientation.x = np.sqrt(2) / 2
            request.ik_request.pose_stamped.pose.orientation.y = np.sqrt(2) / 2
            if i >= 8:
                request.ik_request.pose_stamped.pose.orientation.y *= -1
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0

            # if i == 8 or i== 9:
            #     request.ik_request.pose_stamped.pose.orientation.x = -0.5
            #     request.ik_request.pose_stamped.pose.orientation.y = 0.5
            #     request.ik_request.pose_stamped.pose.orientation.z = 0.5
            #     request.ik_request.pose_stamped.pose.orientation.w = 0.5
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander("right_arm")
                if i != 3:
                    group.set_max_velocity_scaling_factor(0.4)

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                # group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK
                plan = group.plan()
                # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                
                # # Execute IK if safe
                # if user_input == 'y':
                #     group.execute(plan[1])
                group.execute(plan[1])
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            if i == 1 or i == 9:
                right_gripper.close()
                rospy.sleep(2.0)
            elif i == 5 or i == 11:
                right_gripper.open()
                rospy.sleep(2.0)

        const_tower_offset += 0.06

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
