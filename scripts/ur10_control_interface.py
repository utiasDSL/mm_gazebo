#!/usr/bin/env python
# Translate from mm_motion_control to Gazebo's ros_control interface
import rospy

import tf
import tf.transformations as tfs

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import LinkStates


RB_BASELINK_NAME = 'ridgeback::base_link'

UR10_JOINT_NAMES = ['ur10_arm_shoulder_pan_joint',
                    'ur10_arm_shoulder_lift_joint', 'ur10_arm_elbow_joint',
                    'ur10_arm_wrist_1_joint', 'ur10_arm_wrist_2_joint',
                    'ur10_arm_wrist_3_joint']


class UR10ControlInterface(object):
    def __init__(self):
        self.joint_idx = []

        self.rb_position = [0] * 3
        self.rb_velocity = [0] * 3
        self.ur10_position = [0] * 6
        self.ur10_velocity = [0] * 6

        rospy.Subscriber('/ur_driver/joint_speed', JointTrajectory,
                         self.ur10_joint_vel_cb)

        # UR10 joint states are supplied in the /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)

        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb)

        self.cmd_pub = rospy.Publisher('/ur10_velocity_controller/command',
                                       Float64MultiArray, queue_size=10)
        self.joint_state_pub = rospy.Publisher('/mm_joint_states', JointState,
                                               queue_size=10)

    def joint_state_cb(self, msg):
        ''' Translate joint states published by Gazebo to those expected by
            mm_motion_control. '''
        # If we haven't already done so, we need to pick out the relevant
        # joints.
        if len(self.joint_idx) == 0:
            for name in UR10_JOINT_NAMES:
                idx = msg.name.index(name)
                self.joint_idx.append(idx)

        self.ur10_position = [msg.position[i] for i in self.joint_idx]
        self.ur10_velocity = [msg.velocity[i] for i in self.joint_idx]

    def link_states_cb(self, msg):
        # look up the Ridgeback base link
        idx = msg.name.index(RB_BASELINK_NAME)
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # convert from quaternion to Euler angles
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w]
        euler = tfs.euler_from_quaternion(quat)

        self.rb_position = [pose.position.x, pose.position.y, euler[2]]
        self.rb_velocity = [twist.linear.x, twist.linear.y, twist.angular.z]

    def ur10_joint_vel_cb(self, msg):
        ''' Translate velocity commands from mm_motion_control to the
            controllers expected by Gazebo. Only for UR10, since the Ridgeback
            has the same interface. '''
        array_msg = Float64MultiArray()
        array_msg.data = msg.points[0].velocities
        self.cmd_pub.publish(array_msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        msg.effort = [0] * 9
        msg.position = self.rb_position + self.ur10_position
        msg.velocity = self.rb_velocity + self.ur10_velocity

        self.joint_state_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.publish_joint_states()
            rate.sleep()


def main():
    rospy.init_node('ur10_control_interface')

    interface = UR10ControlInterface()
    interface.spin()


if __name__ == '__main__':
    main()
