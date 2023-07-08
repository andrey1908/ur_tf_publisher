#!/usr/bin/env python3

# to successfully import tf2_ros
import sys
sys.path.insert(0, '/home/administrator/Repos/manipulator/geom_ws/devel/lib/python3/dist-packages')

from rtde_receive import RTDEReceiveInterface
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from transforms3d.quaternions import axangle2quat
import numpy as np
from time import sleep


def connect_rtde(ur_ip):
    success = False
    while not success and not rospy.is_shutdown():
        try:
            rtde_r = RTDEReceiveInterface(ur_ip)
        except RuntimeError:
            sleep(1)
        else:
            success = True

    if rospy.is_shutdown():
        return None

    return rtde_r


def publish_arm_base_to_gripper_tf(ur_ip, rate):
    print("Connecting...")
    rtde_r = connect_rtde(ur_ip)
    br = tf2_ros.TransformBroadcaster()
    in_spinning_state = False

    while not rospy.is_shutdown():
        pose = np.array(rtde_r.getActualTCPPose())
        stamp = rospy.Time.now()

        if not rtde_r.isConnected() or pose.shape != (6,):
            if in_spinning_state:
                print("Connecting...")
                in_spinning_state = False
            rtde_r = connect_rtde(ur_ip)
            sleep(1)
            continue

        if not in_spinning_state:
            print("Spinning...")
            in_spinning_state = True

        x, y, z = pose[:3]
        rot_axis = pose[3:]
        angle = np.linalg.norm(rot_axis)
        qw, qx, qy, qz = axangle2quat(rot_axis, angle)

        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = "ur_arm_base"
        tr.child_frame_id = "ur_gripper"
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.translation.z = z
        tr.transform.rotation.x = qx
        tr.transform.rotation.y = qy
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        br.sendTransform(tr)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("arm_base_to_gripper")
    ur_ip = "192.168.131.40"
    rate = rospy.Rate(30)

    publish_arm_base_to_gripper_tf(ur_ip, rate)
