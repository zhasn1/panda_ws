#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub = self.create_publisher(
            JointTrajectory, "arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "gripper_controller/joint_trajectory", 10
        )
        self.sub = self.create_subscription(
            JointState, "joint_commands", self.slider_callback, 10
        )
        self.get_logger().info("Control Node started")

    def slider_callback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()

        arm_controller.joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        gripper_controller.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()

        arm_goal.positions = msg.position[:7]
        gripper_goal.positions = [msg.position[7], msg.position[7]]

        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)

        self.arm_pub.publish(arm_controller)
        self.gripper_pub.publish(gripper_controller)


def main():
    rclpy.init()
    node = SliderControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
