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
from visualization_msgs.msg import MarkerArray, Marker

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
        self.sub = self.create_subscription(MarkerArray, FOXY_VOXBLOX_TOPIC, self.callback, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, PORT))
        self.sock.listen(1)
        self.get_logger().info(f"[Voxblox_Foxy] Waiting for Jazzy client (vS-Graphs) on {host}:{PORT} ...")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"[Voxblox_Foxy] Connected to the client by {addr}!")

    def callback(self, msg: MarkerArray):
        data = {
            "markers": []
        }
        for m in msg.markers:
            data["markers"].append({
                "header": {
                    "stamp": {"sec": m.header.stamp.sec, "nanosec": m.header.stamp.nanosec},
                    "frame_id": m.header.frame_id,
                },
                "ns": m.ns,
                "id": m.id,
                "type": m.type,
                "action": m.action,
                "pose": {
                    "position": {
                        "x": m.pose.position.x,
                        "y": m.pose.position.y,
                        "z": m.pose.position.z,
                    },
                    "orientation": {
                        "x": m.pose.orientation.x,
                        "y": m.pose.orientation.y,
                        "z": m.pose.orientation.z,
                        "w": m.pose.orientation.w,
                    },
                },
                "scale": {
                    "x": m.scale.x,
                    "y": m.scale.y,
                    "z": m.scale.z,
                },
                "color": {
                    "r": m.color.r,
                    "g": m.color.g,
                    "b": m.color.b,
                    "a": m.color.a,
                },
            })
        try:
            self.conn.sendall((json.dumps(data) + "\n").encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("[Voxblox_Foxy] Lost connection to Jazzy (vS-Graphs) client!")


# ---------------------------
# Jazzy side (vS-Graphs): TCP receiver → publisher
# ---------------------------
class JazzyRelay(Node):
    def __init__(self, foxy_host):
        super().__init__('vsgraphs_jazzy_relay')
        self.pub = self.create_publisher(MarkerArray, PUBLISHER_TOPIC, 10)
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
                marker_array = MarkerArray()
                for m in msg_dict.get("markers", []):
                    marker = Marker()
                    marker.header.frame_id = m["header"]["frame_id"]
                    marker.header.stamp.sec = m["header"]["stamp"]["sec"]
                    marker.header.stamp.nanosec = m["header"]["stamp"]["nanosec"]
                    marker.ns = m["ns"]
                    marker.id = m["id"]
                    marker.type = m["type"]
                    marker.action = m["action"]
                    marker.pose.position.x = m["pose"]["position"]["x"]
                    marker.pose.position.y = m["pose"]["position"]["y"]
                    marker.pose.position.z = m["pose"]["position"]["z"]
                    marker.pose.orientation.x = m["pose"]["orientation"]["x"]
                    marker.pose.orientation.y = m["pose"]["orientation"]["y"]
                    marker.pose.orientation.z = m["pose"]["orientation"]["z"]
                    marker.pose.orientation.w = m["pose"]["orientation"]["w"]
                    marker.scale.x = m["scale"]["x"]
                    marker.scale.y = m["scale"]["y"]
                    marker.scale.z = m["scale"]["z"]
                    marker.color.r = m["color"]["r"]
                    marker.color.g = m["color"]["g"]
                    marker.color.b = m["color"]["b"]
                    marker.color.a = m["color"]["a"]
                    marker_array.markers.append(marker)
                self.pub.publish(marker_array)
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
