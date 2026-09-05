#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP image bridge: receives raw RGB frames from Unity over TCP and
republishes them to ROS2 as sensor_msgs/Image + sensor_msgs/CameraInfo.

Frame protocol (little-endian), one frame per write:
    [4B header_len][header JSON][W*H*3 raw rgb8 bytes]

header JSON example:
{
  "w": 640, "h": 480, "step": 1920, "enc": "rgb8",
  "frame_id": "camera_optical_frame",
  "sec": 1788623927, "nsec": 67000000,
  "cam_info": { "distortion_model": "plumb_bob", "d": [...], "k": [...], "r": [...], "p": [...] }
}

Run:
  source /opt/ros/humble/setup.bash
  python3 tcp_image_bridge.py
"""
import json
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

HOST = "0.0.0.0"
PORT = 10001
SIZE = 8  # pre-allocated header len cap (real json is ~700B, but allow growth)


class TcpImageBridge(Node):
    def __init__(self):
        super().__init__("tcp_image_bridge")
        # RELIABLE publisher: compatible with armor_detector(BEST_EFFORT sub) and CLI tools
        self.img_pub = self.create_publisher(Image, "/image_raw", 10)
        self.cam_pub = self.create_publisher(CameraInfo, "/camera_info", 10)
        self.get_logger().info(f"TCP image bridge ready, listening on {HOST}:{PORT}")

    @staticmethod
    def recv_exact(conn, n):
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def handle_client(self, conn, addr):
        self.get_logger().info(f"client connected: {addr}")
        seq = 0
        try:
            while True:
                hb = self.recv_exact(conn, 4)
                if hb is None:
                    break
                (hlen,) = struct.unpack("<I", hb)
                if not (0 < hlen <= 65536):
                    self.get_logger().warning(f"bad header len {hlen}, closing")
                    break
                hb2 = self.recv_exact(conn, hlen)
                if hb2 is None:
                    break
                try:
                    hdr = json.loads(hb2.decode("utf-8"))
                except Exception as e:
                    self.get_logger().warning(f"bad header json: {e}")
                    continue

                w = int(hdr["w"])
                h = int(hdr["h"])
                step = int(hdr.get("step", w * 3))
                frame_bytes = self.recv_exact(conn, step * h)
                if frame_bytes is None:
                    break

                im = Image()
                im.height = h
                im.width = w
                im.encoding = hdr.get("enc", "rgb8")
                im.is_bigendian = 0
                im.step = step
                im.header.frame_id = hdr.get("frame_id", "camera_optical_frame")
                sec = hdr.get("sec")
                nsec = hdr.get("nsec")
                if sec is not None and nsec is not None:
                    im.header.stamp.sec = int(sec)
                    im.header.stamp.nanosec = int(nsec)
                im.data = frame_bytes
                self.img_pub.publish(im)

                ci = hdr.get("cam_info")
                if ci is not None:
                    try:
                        cam = CameraInfo()
                        cam.header.stamp = im.header.stamp
                        cam.header.frame_id = im.header.frame_id
                        cam.height = h
                        cam.width = w
                        cam.distortion_model = ci.get("distortion_model", "plumb_bob")
                        cam.d = [float(x) for x in ci.get("d", [])]
                        cam.k = [float(x) for x in ci.get("k", [])]
                        cam.r = [float(x) for x in ci.get("r", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])]
                        cam.p = [float(x) for x in ci.get("p", [])]
                        cam.binning_x = 0
                        cam.binning_y = 0
                        self.cam_pub.publish(cam)
                    except Exception as ce:
                        self.get_logger().warning(f"camera_info skipped: {ce}")

                seq += 1
                if seq % 100 == 0:
                    self.get_logger().info(
                        f"published {seq} frames ({w}x{h}, {frame_bytes.__len__()}B @ {hdr.get('enc')})"
                    )
        except Exception as e:
            self.get_logger().error(f"client error {addr}: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass
            self.get_logger().info(f"client closed: {addr}")


def main():
    rclpy.init()
    node = TcpImageBridge()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)

    def accept_loop():
        while rclpy.ok():
            try:
                srv.settimeout(1.0)
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except Exception:
                break
            t = threading.Thread(target=node.handle_client, args=(conn, addr), daemon=True)
            t.start()

    t_ac = threading.Thread(target=accept_loop, daemon=True)
    t_ac.start()
    node.get_logger().info("accept loop started")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
