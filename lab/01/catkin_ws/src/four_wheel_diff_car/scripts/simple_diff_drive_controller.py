#!/usr/bin/env python3
import math

import rospy
import tf.transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class SimpleDiffDriveController:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "four_wheel_diff_car")
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.08)
        self.wheel_separation = rospy.get_param("~wheel_separation", 0.41)
        self.control_rate = rospy.get_param("~control_rate", 30.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        self.x = rospy.get_param("~initial_x", 0.0)
        self.y = rospy.get_param("~initial_y", 0.0)
        self.yaw = rospy.get_param("~initial_yaw", 0.0)
        self.z = rospy.get_param("~initial_z", 0.12)

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = rospy.Time.now()

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)

        rospy.loginfo("waiting for /gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("simple diff drive controller is ready")

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def active_command(self):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return 0.0, 0.0
        return self.last_cmd.linear.x, self.last_cmd.angular.z

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0.0:
            return

        linear, angular = self.active_command()

        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw += angular * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        left_speed = (linear - angular * self.wheel_separation / 2.0) / self.wheel_radius
        right_speed = (linear + angular * self.wheel_separation / 2.0) / self.wheel_radius
        self.left_wheel_pos += left_speed * dt
        self.right_wheel_pos += right_speed * dt

        self.publish_joint_states(now)
        self.publish_model_state(linear, angular)

    def publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        msg.position = [
            self.left_wheel_pos,
            self.right_wheel_pos,
            self.left_wheel_pos,
            self.right_wheel_pos,
        ]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(msg)

    def publish_model_state(self, linear, angular):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)

        state = ModelState()
        state.model_name = self.model_name
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]
        state.twist.linear.x = linear
        state.twist.angular.z = angular
        state.reference_frame = "world"

        try:
            self.set_model_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "failed to update model state: %s", exc)

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    rospy.init_node("simple_diff_drive_controller")
    SimpleDiffDriveController().spin()


if __name__ == "__main__":
    main()
