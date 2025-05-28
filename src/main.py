import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
from src.functions.jacobian import control_step

class JacobianControl(Node):
    def __init__(self):
        super().__init__('jacobian_control_node')
        self.goal_pose = None
        self.current_joints = None

        self.pose_sub = self.create_subscription(
            Pose, 'goal/pose', self.goal_callback, 10)

        self.joint_sub = self.create_subscription(
            JointState, 'denso/joint_states', self.joint_callback, 10)

        self.publisher = self.create_publisher(
            Float64MultiArray, 'denso/target_positions', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def goal_callback(self, msg):
        pos = msg.position
        ori = msg.orientation
        self.goal_pose = np.array([pos.x, pos.y, pos.z,
                                   ori.x, ori.y, ori.z])  # quat simplificado

    def joint_callback(self, msg):
        self.current_joints = np.array(msg.position)

    def timer_callback(self):
        if self.goal_pose is None or self.current_joints is None:
            return

        new_q = control_step(self.current_joints, self.goal_pose)
        msg = Float64MultiArray()
        msg.data = new_q.tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JacobianControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
