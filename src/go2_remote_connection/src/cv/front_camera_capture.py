#!/usr/bin/env python3
import os
import socket
import struct
import time

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
        if sent <= 0:
            raise ConnectionError("Socket send failed")
        view = view[sent:]


def main():
    print("=== CAPTURE ENV CHECK ===", flush=True)
    for k in ["UNITREE_IFACE", "CYCLONEDDS_URI", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID"]:
        print(f"{k} = {os.environ.get(k)}", flush=True)
    print("=========================", flush=True)

    iface = choose_unitree_iface()
    print(f"Initializing Unitree channel on iface: {iface}", flush=True)

    ChannelFactoryInitialize(0, iface)

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    print("Video client initialized successfully", flush=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Waiting for ROS bridge on {HOST}:{PORT} ...", flush=True)
    conn, addr = server.accept()
    print(f"ROS bridge connected from {addr}", flush=True)

    frame_count = 0
    last_log_time = time.time()

    try:
        while True:
            code, data = client.GetImageSample()
            if code != 0 or data is None:
                time.sleep(0.002)
                continue

            # IMPORTANT:
            # Unitree already gives us compressed image bytes.
            # Do NOT decode + re-encode here; just forward them directly.
            payload = bytes(data)
            if not payload:
                continue

            header = struct.pack("!I", len(payload))
            send_all(conn, header)
            send_all(conn, payload)

            frame_count += 1
            now = time.time()
            if now - last_log_time >= 5.0:
                fps = frame_count / (now - last_log_time)
                print(f"[capture] streaming at ~{fps:.1f} FPS", flush=True)
                frame_count = 0
                last_log_time = now

    except KeyboardInterrupt:
        print("Capture interrupted", flush=True)
    except BrokenPipeError:
        print("ROS bridge disconnected (broken pipe)", flush=True)
    except ConnectionResetError:
        print("ROS bridge disconnected (connection reset)", flush=True)
    except Exception as e:
        print(f"Capture error: {e}", flush=True)
    finally:
        try:
            conn.close()
        except Exception:
            pass
        try:
            server.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()