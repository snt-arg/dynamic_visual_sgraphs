#!/usr/bin/env python3

"""
* This file is part of Visual S-Graphs (vS-Graphs).
* Copyright (C) 2023-2025 SnT, University of Luxembourg
*
* 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
*
* vS-Graphs is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
*
* This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details: https://www.gnu.org/licenses/
"""

import json
import rclpy
import socket
import argparse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Variables
PORT = 5000
PUBLISHER_TOPIC = "/vox2ros/sparse_graph"
FOXY_VOXBLOX_TOPIC = "/voxblox_skeletonizer/sparse_graph"

# ---------------------------
# Foxy side (Voxblox): subscriber → TCP sender
# ---------------------------
class FoxyRelay(Node):
    def __init__(self, host="0.0.0.0"):
        super().__init__('voxblox_foxy_relay')
        self.sub = self.create_subscription(PoseStamped, FOXY_VOXBLOX_TOPIC, self.callback, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, PORT))
        self.sock.listen(1)
        self.get_logger().info(f"[Voxblox_Foxy] Waiting for Jazzy client (vS-Graphs) on {host}:{PORT} ...")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"[Voxblox_Foxy] Connected to the client by {addr}!")

    def callback(self, msg: PoseStamped):
        data = {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "pose": {
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            },
        }
        try:
            self.conn.sendall((json.dumps(data) + "\n").encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("[Voxblox_Foxy] Lost connection to Jazzy (vS-Graphs) client!")


# ---------------------------
# Jazzy side (vS-Graphs): TCP receiver → publisher
# ---------------------------
class JazzyRelay(Node):
    def __init__(self, foxy_host):
        super().__init__('vsgraphs_jazzy_relay')
        self.pub = self.create_publisher(PoseStamped, PUBLISHER_TOPIC, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((foxy_host, PORT))
        self.sock.setblocking(False)
        self.buffer = ""
        self.get_logger().info(f"[vS-Graphs Jazzy] Connected to Foxy relay at {foxy_host}:{PORT}")
        self.create_timer(0.01, self.receive_loop)

    def receive_loop(self):
        try:
            chunk = self.sock.recv(4096).decode('utf-8')
            self.buffer += chunk
        except BlockingIOError:
            pass

        # process complete JSON lines
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if not line.strip():
                continue
            try:
                msg_dict = json.loads(line)
                msg = PoseStamped()
                msg.header.stamp.sec = msg_dict["header"]["stamp"]["sec"]
                msg.header.stamp.nanosec = msg_dict["header"]["stamp"]["nanosec"]
                msg.header.frame_id = msg_dict["header"]["frame_id"]
                msg.pose.position.x = msg_dict["pose"]["position"]["x"]
                msg.pose.position.y = msg_dict["pose"]["position"]["y"]
                msg.pose.position.z = msg_dict["pose"]["position"]["z"]
                msg.pose.orientation.x = msg_dict["pose"]["orientation"]["x"]
                msg.pose.orientation.y = msg_dict["pose"]["orientation"]["y"]
                msg.pose.orientation.z = msg_dict["pose"]["orientation"]["z"]
                msg.pose.orientation.w = msg_dict["pose"]["orientation"]["w"]
                self.pub.publish(msg)
            except json.JSONDecodeError:
                self.get_logger().warn("[vS-Graphs Jazzy] Failed to decode JSON line")


# ---------------------------
# Main launcher
# ---------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["voxblox_foxy", "vsgraphs_jazzy"], required=True,
                        help="Run as Voxblox Foxy relay (server) or vS-Graphs Jazzy relay (client)!")
    parser.add_argument("--foxy-host", default="127.0.0.1",
                        help="IP address of Foxy container (for Jazzy mode)!")
    args = parser.parse_args()

    rclpy.init()

    if args.mode == "voxblox_foxy":
        node = FoxyRelay()
    elif args.mode == "vsgraphs_jazzy":
        node = JazzyRelay(args.foxy_host)
    else:
        raise ValueError("Invalid mode! Use 'voxblox_foxy' or 'vsgraphs_jazzy'.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
