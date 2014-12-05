#!/usr/bin/env python

from transformation_helper import *
import signal, os
import argparse
import struct
import sys
import rospy
import math
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
    Wrench
)
from gazebo_msgs.msg import (
    WorldState
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from baxter_core_msgs.msg import (
    EndpointState
)

# right input
rightInputX = .6
rightInputY = 0.1
rightInputZ = 0.1

rightInputXR = 0.37
rightInputYR = 0.92
rightInputZR = 0.1
rightInputWR = 0.02

# left input
leftInputX = 0.6
leftInputY = 0.4
leftInputZ = 0.5

leftInputXR = -0.4
leftInputYR = 0.92
leftInputZR = 0
leftInputWR = 0.02
global latest_pose
callback_flag = False
global goto
class execute_trajectory:
    #right_arm_pose_pub = rospy.Publisher('/right_arm_pose_controller/target', PoseStamped, queue_size=10)
    #left_arm_pose_pub = None
    timeout = signal.SIG_IGN
    latest_right_arm_pose = None
    rot_error_limit = 0.1
    trans_error_limit = 0.2
    def __init__(self):
        self.right_arm_pose_pub = rospy.Publisher('/right_arm_pose_controller/target', PoseStamped, latch=True)
        self.left_arm_pose_pub = rospy.Publisher('/left_arm_pose_controller/target', PoseStamped, queue_size=10)
        print "instance created"

    def setLatestRightArmPose(self):
        self.latest_right_arm_pose = latest_pose

    def execute_to_pose_target(self, target_pose, exec_time_limit):
        # Ensure that the exec_time_limit is an integer, since SIGALRM can only take integer arguments
        exec_time_limit = int(math.ceil(exec_time_limit))
        print "publishing execute to pose"
        #while self.right_arm_pose_pub.getNumSubscribers == 0:
        #    rospy.sleep(100)
        #print "out of the sleep loop"
        self.right_arm_pose_pub.publish(target_pose)

        print "after loop"
        rospy.sleep(3)
        print target_pose
        # Set timer
        signal.signal(signal.SIGALRM, self.timeout)
        signal.alarm(exec_time_limit)
        # Check to see if we've reached the target
        while not self.pose_target_reached(target_pose):
            pass
        print "Broke out of while loop"
        # Now that we've converged, stop timer
        signal.alarm(0)
        # Stop the pose controller
        cancel_pose = PoseStamped()
        cancel_pose.header.frame_id = "/torso_lift_link"
        cancel_pose.pose.orientation.x = 0.0
        cancel_pose.pose.orientation.y = 0.0
        cancel_pose.pose.orientation.z = 0.0
        cancel_pose.pose.orientation.w = 0.0
        self.right_arm_pose_pub.publish(cancel_pose)
        print "end of execute pose"
        return True


    def pose_target_reached(self, target_pose):
        working_current_pose = self.latest_right_arm_pose
        pose_error = self.compute_pose_error(target_pose, working_current_pose)
        trans_error_magnitude = linalg.norm(numpy.array([pose_error[0], pose_error[1], pose_error[2]]))
        rot_error_magnitude = linalg.norm(numpy.array([pose_error[3], pose_error[4], pose_error[5]]))
        #print "CPC trans error magnitude: " + str(trans_error_magnitude)
        #print "CPC rot error magnitude: " + str(rot_error_magnitude)
        if (fabs(trans_error_magnitude) <= self.trans_error_limit) and (fabs(rot_error_magnitude) <= self.rot_error_limit):
            return True
        else:
            return False

    def compute_pose_error(self, target_pose, current_pose):
        # Convert poses to matrix form
        target_pose_matrix = PoseToMatrix(target_pose.pose)
        current_pose_matrix = PoseToMatrix(current_pose.pose)
        # Extract the translation vector + rotation matrix from each
        [target_pose_rot, target_pose_trans] = ExtractRawFromMatrix(target_pose_matrix)
        [current_pose_rot, current_pose_trans] = ExtractRawFromMatrix(current_pose_matrix)
        # Compute the translation error
        xt_err = target_pose_trans[0] - current_pose_trans[0]
        yt_err = target_pose_trans[1] - current_pose_trans[1]
        zt_err = target_pose_trans[2] - current_pose_trans[2]
        # Compute the rotation error - first, get the columns of the rotation matrices
        tprc1 = numpy.array([target_pose_rot[0][0], target_pose_rot[1][0], target_pose_rot[2][0]])
        tprc2 = numpy.array([target_pose_rot[0][1], target_pose_rot[1][1], target_pose_rot[2][1]])
        tprc3 = numpy.array([target_pose_rot[0][2], target_pose_rot[1][2], target_pose_rot[2][2]])
        cprc1 = numpy.array([current_pose_rot[0][0], current_pose_rot[1][0], current_pose_rot[2][0]])
        cprc2 = numpy.array([current_pose_rot[0][1], current_pose_rot[1][1], current_pose_rot[2][1]])
        cprc3 = numpy.array([current_pose_rot[0][2], current_pose_rot[1][2], current_pose_rot[2][2]])
        # Second, cross the columns together
        rot_err = multiply(cross(tprc1, cprc1) + cross(tprc2, cprc2) + cross(tprc3, cprc3), -0.5)
        # Combine together
        pose_error = numpy.array([xt_err, yt_err, zt_err, rot_err[0], rot_err[1], rot_err[2]])
        return pose_error

def callback(msg):
    global latest_pose, goto, callback_flag
    latest_pose = msg
    goto.setLatestRightArmPose()
    callback_flag = True
    #if callback_flag:
    #    print "callback end"



def main():
    global goto, callback_flag
    rospy.init_node("ik_goal_publisher")
    goto = execute_trajectory()
    rospy.Subscriber("/right_arm_pose_controller/state", PoseStamped, callback)
    print "EVERYTHING SET"

    while True:
        if callback_flag == True:
            print "callback accepted"

            hdr = Header(stamp=rospy.Time.now(), frame_id='base')

            leftPose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=leftInputX,
                        y=leftInputY,
                        z=leftInputZ,
                    ),
                    orientation=Quaternion(
                        x=leftInputXR,
                        y=leftInputYR,
                        z=leftInputZR,
                        w=leftInputWR,
                    ),
                ),
            )

            rightPose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=rightInputX,
                        y=rightInputY,
                        z=rightInputZ,
                    ),
                    orientation=Quaternion(
                        x=rightInputXR,
                        y=rightInputYR,
                        z=rightInputZR,
                        w=rightInputWR,
                    ),
                ),
            )

            arg_fmt = argparse.RawDescriptionHelpFormatter
            parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                             description=main.__doc__)
            parser.add_argument(
                '-l', '--limb', choices=['left', 'right'], required=True,
                help="the limb to test"
            )
            args = parser.parse_args(rospy.myargv()[1:])

            if args.limb == 'left':

                print "publishing left hand..."
                #leftHandPub.publish(leftPose)
                rospy.sleep(.1)

            if args.limb == 'right':

                print "publishing right hand..."
                #rightHandPub.publish(rightPose)
                goto.execute_to_pose_target(rightPose, 30)
                #goto.right_arm_pose_pub.publish(rightPose)
                rospy.sleep(.1)

    return 0


if __name__ == '__main__':
    main()
    rospy.spin()
