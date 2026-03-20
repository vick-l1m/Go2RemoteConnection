#!/usr/bin/env python3
import os
import socket
import struct
import time

import cv2
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


HOST = "127.0.0.1"
PORT = 9998


def choose_unitree_iface():
    iface = os.environ.get("UNITREE_IFACE", "").strip()
    if not iface:
        raise RuntimeError("UNITREE_IFACE is not set")
    return iface


def send_all(sock: socket.socket, data: bytes) -> None:
    view = memoryview(data)
    while view:
        sent = sock.send(view)
        view = view[sent:]


def main():
    print("=== CAPTURE ENV CHECK ===")
    for k in ["UNITREE_IFACE", "CYCLONEDDS_URI", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID"]:
        print(f"{k} = {os.environ.get(k)}")
    print("=========================")

    iface = choose_unitree_iface()
    print(f"Initializing Unitree channel on iface: {iface}")

    ChannelFactoryInitialize(0, iface)

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    print("Video client initialized successfully")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Waiting for ROS bridge on {HOST}:{PORT} ...")
    conn, addr = server.accept()
    print(f"ROS bridge connected from {addr}")

    try:
        while True:
            code, data = client.GetImageSample()
            if code != 0 or data is None:
                time.sleep(0.01)
                continue

            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is None:
                continue

            ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                continue

            payload = encoded.tobytes()
            header = struct.pack("!I", len(payload))
            send_all(conn, header)
            send_all(conn, payload)

    except KeyboardInterrupt:
        print("Capture interrupted")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        server.close()


if __name__ == "__main__":
    main()