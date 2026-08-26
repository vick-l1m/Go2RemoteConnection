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

JPEG_QUALITY = int(os.environ.get("GO2_CAM_JPEG_QUALITY", "50"))
SCALE = float(os.environ.get("GO2_CAM_SCALE", "1.0"))


def choose_unitree_iface():
    iface = os.environ.get("UNITREE_IFACE", "").strip()
    if not iface:
        raise RuntimeError("UNITREE_IFACE is not set")
    return iface


def send_all(sock: socket.socket, data: bytes) -> None:
    view = memoryview(data)
    while view:
        sent = sock.send(view)
        if sent <= 0:
            raise ConnectionError("Socket send failed")
        view = view[sent:]


def main():
    iface = choose_unitree_iface()
    ChannelFactoryInitialize(0, iface)

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    conn, _ = server.accept()

    try:
        while True:
            code, data = client.GetImageSample()
            if code != 0 or data is None:
                time.sleep(0.002)
                continue

            raw = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if image is None:
                continue

            if SCALE != 1.0:
                h, w = image.shape[:2]
                image = cv2.resize(
                    image,
                    (max(1, int(w * SCALE)), max(1, int(h * SCALE))),
                    interpolation=cv2.INTER_AREA
                )

            ok, encoded = cv2.imencode(
                ".jpg",
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            )
            if not ok:
                continue

            payload = encoded.tobytes()
            header = struct.pack("!I", len(payload))
            send_all(conn, header)
            send_all(conn, payload)

    finally:
        try:
            conn.close()
        except Exception:
            pass
        server.close()


if __name__ == "__main__":
    main()