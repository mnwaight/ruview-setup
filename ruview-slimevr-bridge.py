#!/usr/bin/env python3
"""
ruview-slimevr-bridge.py
Bridges ruview WiFi pose data to SlimeVR virtual trackers.

Reads 17-keypoint COCO pose from ruview WebSocket and sends
quaternion rotation packets to SlimeVR server via UDP.

Supports two simultaneous players — ruview tracks both people
and each gets their own independent set of virtual trackers.

Usage:
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.X
  python3 ruview-slimevr-bridge.py --ruview-ws ws://192.168.12.150:3001/ws/sensing --slimevr-host 192.168.12.50
"""

import asyncio
import json
import math
import socket
import struct
import argparse
import websockets

PACKET_HANDSHAKE     = 3
PACKET_ROTATION_DATA = 17

TRACKER_HIP         = 0
TRACKER_CHEST       = 1
TRACKER_LEFT_THIGH  = 2
TRACKER_RIGHT_THIGH = 3
TRACKER_LEFT_SHIN   = 4
TRACKER_RIGHT_SHIN  = 5

TRACKER_COUNT = 6


# ── Math helpers ──────────────────────────────────────────────────────────────

def normalize(v):
    mag = math.sqrt(sum(x * x for x in v))
    return tuple(x / mag for x in v) if mag > 1e-6 else (0.0, 1.0, 0.0)

def cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )

def subtract(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def midpoint(a, b):
    return ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2)

def vec_to_quaternion(direction):
    """Convert a direction vector to a quaternion (w,x,y,z)."""
    fwd = normalize(direction)
    world_up = (0.0, 1.0, 0.0)
    right = normalize(cross(world_up, fwd))
    up = cross(fwd, right)

    m00, m01, m02 = right
    m10, m11, m12 = up
    m20, m21, m22 = fwd

    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return (x, y, z, w)


# ── Keypoint extraction ───────────────────────────────────────────────────────

def extract_keypoints(person):
    """Pull COCO keypoints into a dict, remapping to VR coordinate space."""
    kps = {}
    for kp in person.get('keypoints', []):
        name = kp['name']
        # ruview: x=right, y=up(screen), z=depth → VR: x=right, y=up, z=forward
        kps[name] = (kp['x'], kp['y'], kp.get('z', 0.0))
    return kps

def compute_trackers(kps):
    """Derive quaternion for each virtual tracker from keypoint positions."""
    def get(name):
        return kps.get(name, (0.5, 0.5, 0.0))

    lhip      = get('left_hip')
    rhip      = get('right_hip')
    lshoulder = get('left_shoulder')
    rshoulder = get('right_shoulder')
    lknee     = get('left_knee')
    rknee     = get('right_knee')
    lankle    = get('left_ankle')
    rankle    = get('right_ankle')

    hip_mid      = midpoint(lhip, rhip)
    shoulder_mid = midpoint(lshoulder, rshoulder)
    spine_vec    = subtract(shoulder_mid, hip_mid)

    return {
        TRACKER_HIP:         vec_to_quaternion(spine_vec),
        TRACKER_CHEST:       vec_to_quaternion(spine_vec),
        TRACKER_LEFT_THIGH:  vec_to_quaternion(subtract(lknee, lhip)),
        TRACKER_RIGHT_THIGH: vec_to_quaternion(subtract(rknee, rhip)),
        TRACKER_LEFT_SHIN:   vec_to_quaternion(subtract(lankle, lknee)),
        TRACKER_RIGHT_SHIN:  vec_to_quaternion(subtract(rankle, rknee)),
    }


# ── SlimeVR UDP sender ────────────────────────────────────────────────────────

class SlimeVRSender:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packet_num = 0

    def _pnum(self):
        self.packet_num += 1
        return self.packet_num

    def _mac(self, person_id, tracker_id):
        # Unique fake MAC per person+tracker so SlimeVR treats each as distinct hardware
        return bytes([0x52, 0x75, 0x56, 0x57, person_id & 0xFF, tracker_id & 0xFF])

    def send_handshake(self, person_id, tracker_id):
        mac = self._mac(person_id, tracker_id)
        firmware = b'ruview-bridge\x00'
        pkt  = struct.pack('>I', PACKET_HANDSHAKE)
        pkt += struct.pack('>Q', self._pnum())
        pkt += struct.pack('>IIIIII', 0, 0, 0, 0, 0, 0)  # board/imu/mcu/info
        pkt += firmware
        pkt += mac
        self.sock.sendto(pkt, (self.host, self.port))

    def send_rotation(self, person_id, tracker_id, quat):
        x, y, z, w = quat
        pkt  = struct.pack('>I', PACKET_ROTATION_DATA)
        pkt += struct.pack('>Q', self._pnum())
        pkt += struct.pack('B', (person_id * TRACKER_COUNT) + tracker_id)
        pkt += struct.pack('B', 1)           # data type: normal
        pkt += struct.pack('>ffff', x, y, z, w)
        pkt += struct.pack('B', 0)           # accuracy
        self.sock.sendto(pkt, (self.host, self.port))


# ── Bridge loop ───────────────────────────────────────────────────────────────

async def run_bridge(ruview_ws_url, slimevr_host, slimevr_port, api_token=None):
    sender   = SlimeVRSender(slimevr_host, slimevr_port)
    handshook = set()

    headers = {}
    if api_token:
        headers['Authorization'] = f'Bearer {api_token}'

    print(f"Connecting to ruview  → {ruview_ws_url}")
    print(f"Forwarding to SlimeVR → {slimevr_host}:{slimevr_port}")
    print("Waiting for pose data...")

    async with websockets.connect(ruview_ws_url, additional_headers=headers) as ws:
        print("Connected.\n")
        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            for person in data.get('persons', []):
                pid      = person.get('id', 0)
                kps      = extract_keypoints(person)
                trackers = compute_trackers(kps)

                for tid, quat in trackers.items():
                    key = (pid, tid)
                    if key not in handshook:
                        sender.send_handshake(pid, tid)
                        handshook.add(key)
                        await asyncio.sleep(0.05)
                        print(f"  Registered tracker person={pid} tracker={tid}")

                    sender.send_rotation(pid, tid, quat)


def main():
    parser = argparse.ArgumentParser(description='ruview → SlimeVR body tracking bridge')
    parser.add_argument(
        '--ruview-ws',
        default='ws://192.168.12.150:3001/ws/sensing',
        help='ruview WebSocket URL (default: ws://192.168.12.150:3001/ws/sensing)',
    )
    parser.add_argument(
        '--slimevr-host',
        required=True,
        help='IP address of the PC running SlimeVR server',
    )
    parser.add_argument(
        '--slimevr-port',
        type=int,
        default=6969,
        help='SlimeVR UDP port (default: 6969)',
    )
    parser.add_argument(
        '--api-token',
        default='a7f3d2e1-9b4c-4f8a-b6e2-3d5c1a0f7e94',
        help='ruview API bearer token',
    )
    args = parser.parse_args()

    asyncio.run(run_bridge(args.ruview_ws, args.slimevr_host, args.slimevr_port, args.api_token))


if __name__ == '__main__':
    main()
