import rclpy
import threading

from rclpy.node import Node
from rclpy.experimental.events_executor import EventsExecutor

from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

class TrajectoryGeneratorNode:
    """
    A node that generates trajectories for the dynamixel motors.
    """

    def __init__(self):
        rclpy.init()
        self.node = Node("trajectory_generator")

        # Create own executor for Python part
        executor = EventsExecutor()
        executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=executor.spin, args=(), daemon=True)
        self.spin_thread.start()

        self.node.create_subscription(JointCommand, "/walking_motor_goals", self.joint_command_callback, 1) 

        self.publisher_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 1)
        self.publisher_clock = self.node.create_publisher(Clock, "/clock", 1)
        self.publisher_joint_state = self.node.create_publisher(JointState, "/joint_states", 1)

        self.latest_motor_goals = None

    def calculate_trajectory(self, clock: Clock, cmd_vel: Twist, joint_states: JointState):
        """
        Calculate the trajectory at time t.
        """
        self.publisher_cmd_vel.publish(cmd_vel)
        self.publisher_joint_state.publish(joint_states)
        self.publisher_clock.publish(clock)
        
        return self.latest_motor_goals

    def joint_command_callback(self, msg: JointCommand):
        self.latest_motor_goals = msg

    def shutdown(self):
        rclpy.shutdown()
        self.spin_thread.join()
