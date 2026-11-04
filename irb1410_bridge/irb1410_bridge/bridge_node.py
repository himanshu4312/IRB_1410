import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import math
import time

ROBOT_IP = "10.1.22.44"
ROBOT_PORT = 7000

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

class BridgeNode(Node):
    def __init__(self):
        super().__init__('irb1410_bridge')

        # Subscribe to slider joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        # Publish actual feedback
        self.publisher = self.create_publisher(
            JointState, '/joint_states_actual', 10)

        # Store last commanded and target angles in degrees
        self.current_deg = [0.0] * 6
        self.target_deg = [0.0] * 6

        # Max step for joints 4–6 (deg per timer tick)
        self.max_step_wrist = 1  # 0.5 degree per update

        # Timer at 20 Hz to send smooth commands
        self.timer_period = 0.1  # 50 ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Bridge node started. Listening to /joint_states...")

    def joint_callback(self, msg: JointState):
        """
        Called whenever the slider publishes /joint_states.
        Here we only update target_deg (desired angles).
        Actual sending happens in timer_callback().
        """
        angle_dict = {}
        for i, name in enumerate(msg.name):
            angle_dict[name] = msg.position[i]

        if not all(j in angle_dict for j in JOINT_NAMES):
            return

        # Convert ROS (rad) to degrees
        j1 = math.degrees(angle_dict["joint1"])
        j2 = math.degrees(angle_dict["joint2"])
        j3 = math.degrees(angle_dict["joint3"])
        j4 = math.degrees(angle_dict["joint4"])
        j5 = math.degrees(angle_dict["joint5"])
        j6 = math.degrees(angle_dict["joint6"])

        # IRB1410 kinematic correction for joint3 (j3 + j2)
        j2_deg = j2
        j3_deg = j3 + j2_deg

        self.target_deg = [
            round(j1, 2),
            round(j2_deg, 2),
            round(j3_deg, 2),
            round(j4, 2),
            round(j5, 2),
            round(j6, 2),
        ]

    def timer_callback(self):
        """
        Runs at fixed rate (20 Hz).
        Gradually moves current_deg toward target_deg,
        limiting the step for joints 4–6.
        Then sends one command string to the robot.
        """
        # If nothing changed, do nothing
        if self.current_deg == self.target_deg:
            return

        new_cmd = self.current_deg.copy()

        for i in range(6):
            target = self.target_deg[i]
            current = self.current_deg[i]

            diff = target - current

            if abs(diff) < 1e-2:
                # Very small difference, snap
                new_cmd[i] = target
                continue

            if i < 3:
                # joints 1–3: no extra limit, move fully each step
                new_cmd[i] = target
            else:
                # joints 4–6: limit change to max_step_wrist
                step = self.max_step_wrist
                if diff > step:
                    new_cmd[i] = current + step
                elif diff < -step:
                    new_cmd[i] = current - step
                else:
                    new_cmd[i] = target

        self.current_deg = new_cmd

        cmd = ",".join(f"{a:.2f}" for a in self.current_deg)
        self.get_logger().info(f"Sending: {cmd}")
        self.send_to_robot(cmd)

    def send_to_robot(self, cmd: str):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)
                s.connect((ROBOT_IP, ROBOT_PORT))
                s.sendall(cmd.encode())
                feedback = s.recv(1024).decode().strip()
                if feedback:
                    self.publish_feedback(feedback)
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")

    def publish_feedback(self, feedback: str):
        try:
            vals_deg = [float(v) for v in feedback.split(",")]
            vals_rad = [math.radians(v) for v in vals_deg]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = vals_rad
            self.publisher.publish(msg)
            self.get_logger().info(f"Feedback received: {feedback}")
        except Exception as e:
            self.get_logger().error(f"Feedback parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()