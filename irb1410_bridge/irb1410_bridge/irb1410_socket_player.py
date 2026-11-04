#!/usr/bin/env python3

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


HOST = '0.0.0.0'    # listen on all interfaces
PORT = 7000         # must match your master Python sender


# mapping commands -> target joint2, joint3, joint5 in DEGREES
CMD_TO_DEG = {
    'HOME': (0.0, 0.0, 0.0),
    'POS1': (0.0, 0.0, 30.0),
    'POS2': (30.0, 40.0, 0.0),
    'POS3': (60.0, 0.0, 0.0),
    'POS4': (-30.0, 30.0, 0.0),
}

# approximate duration (seconds) for each move
# tune these to match the real robot speed visually
CMD_TO_DURATION = {
    'HOME': 3.0,
    'POS1': 3.0,
    'POS2': 4.0,
    'POS3': 4.0,
    'POS4': 3.5,
}


class IRB1410SocketPlayer(Node):
    def __init__(self):
        super().__init__('irb1410_socket_player')

        # joint names must match URDF
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'ef_finger_r',
            'ef_finger_l',
        ]

        # current joint positions (radians / meters)
        self.current_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
            'ef_finger_r': 0.0,
            'ef_finger_l': 0.0,
        }

        # interpolation state
        self.lock = threading.Lock()
        self.motion_active = False
        self.start_time = 0.0
        self.duration = 3.0  # default seconds
        self.start_j2 = 0.0
        self.start_j3 = 0.0
        self.start_j5 = 0.0
        self.target_j2 = 0.0
        self.target_j3 = 0.0
        self.target_j5 = 0.0

        # publisher to /joint_states
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # timer: 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        # start TCP server in separate thread
        self.server_thread = threading.Thread(
            target=self.socket_server_loop,
            daemon=True
        )
        self.server_thread.start()

        self.get_logger().info(
            f'IRB1410SocketPlayer started. Listening on {HOST}:{PORT}'
        )

    # ------------ TCP SERVER --------------

    def socket_server_loop(self):
        """Simple blocking TCP server with command queue."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        self.get_logger().info('TCP server listening...')

        cmd_queue = []

        def client_handler(conn, addr):
            self.get_logger().info(f'Client connected: {addr}')
            try:
                data = conn.recv(1024)
                if not data:
                    return
                cmd = data.decode('utf-8').strip().upper()
                self.get_logger().info(f'Received command: {cmd}')
                if cmd in CMD_TO_DEG:
                    cmd_queue.append(cmd)
                else:
                    self.get_logger().warn(f'Unknown command: {cmd}')
            except Exception as e:
                self.get_logger().error(f'Error in client_handler: {e}')
            finally:
                conn.close()

        while rclpy.ok():
            try:
                conn, addr = s.accept()
            except OSError:
                break
            t = threading.Thread(
                target=client_handler,
                args=(conn, addr),
                daemon=True
            )
            t.start()

            # process queue: if no active motion, start next one
            with self.lock:
                if (not self.motion_active) and cmd_queue:
                    next_cmd = cmd_queue.pop(0)
                    self.start_motion_to_cmd(next_cmd)

        s.close()

    # ----------- MOTION CONTROL -----------

    def start_motion_to_cmd(self, cmd: str):
        """Set up interpolation from current pose to target pose for the cmd."""
        j2_deg, j3_deg, j5_deg = CMD_TO_DEG[cmd]

        # degrees -> radians conversion
        j2 = math.radians(j2_deg)
        j3 = math.radians(j3_deg)
        j5 = math.radians(j5_deg)

        # read current positions
        self.start_j2 = self.current_positions['joint2']
        self.start_j3 = self.current_positions['joint3']
        self.start_j5 = self.current_positions['joint5']

        self.target_j2 = j2
        self.target_j3 = j3
        self.target_j5 = j5

        # duration for this command
        self.duration = CMD_TO_DURATION.get(cmd, 3.0)
        self.start_time = time.time()
        self.motion_active = True

        self.get_logger().info(
            f'Starting motion to {cmd}: '
            f'j2={j2_deg}deg, j3={j3_deg}deg, j5={j5_deg}deg, '
            f'duration={self.duration}s'
        )

    # ------------ TIMER / PUBLISH ---------

    def timer_callback(self):
        now = time.time()

        with self.lock:
            if self.motion_active:
                t = (now - self.start_time) / self.duration
                if t >= 1.0:
                    # reached target
                    self.current_positions['joint2'] = self.target_j2
                    self.current_positions['joint3'] = self.target_j3
                    self.current_positions['joint5'] = self.target_j5
                    self.motion_active = False
                else:
                    # linear interpolation
                    self.current_positions['joint2'] = (
                        self.start_j2 + t * (self.target_j2 - self.start_j2)
                    )
                    self.current_positions['joint3'] = (
                        self.start_j3 + t * (self.target_j3 - self.start_j3)
                    )
                    self.current_positions['joint5'] = (
                        self.start_j5 + t * (self.target_j5 - self.start_j5)
                    )

        # publish JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions = []
        for name in self.joint_names:
            positions.append(self.current_positions.get(name, 0.0))
        msg.position = positions

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IRB1410SocketPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()