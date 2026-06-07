#!/usr/bin/env python3
"""
ruview-slimevr-bridge.py
Bridges ruview WiFi pose data to SlimeVR virtual trackers.

Reads 17-keypoint COCO pose from ruview WebSocket and sends
quaternion rotation packets to SlimeVR server via UDP.

Supports two simultaneous players with T-pose calibration —
each player strikes a T-pose to claim their person ID before
the session starts, preventing interference from bystanders.

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
import time
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

# T-pose calibration settings
TPOSE_HOLD_SECONDS  = 2.0   # how long the pose must be held
TPOSE_ARM_THRESHOLD = 0.12  # max vertical deviation of wrists vs shoulders (normalized)
TPOSE_EXT_THRESHOLD = 0.15  # min horizontal extension of wrists beyond shoulders


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
    kps = {}
    for kp in person.get('keypoints', []):
        name = kp['name']
        kps[name] = (kp['x'], kp['y'], kp.get('z', 0.0))
    return kps

def compute_trackers(kps):
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


# ── T-pose detection ──────────────────────────────────────────────────────────

def is_tpose(kps):
    """
    Returns True if keypoints match a T-pose:
    - Both wrists at roughly the same height as shoulders
    - Both wrists extended outward beyond shoulders horizontally
    """
    def get(name):
        return kps.get(name)

    lshoulder = get('left_shoulder')
    rshoulder = get('right_shoulder')
    lwrist    = get('left_wrist')
    rwrist    = get('right_wrist')

    if not all([lshoulder, rshoulder, lwrist, rwrist]):
        return False

    # Check wrists are at shoulder height (y axis, normalized 0-1)
    left_y_diff  = abs(lwrist[1] - lshoulder[1])
    right_y_diff = abs(rwrist[1] - rshoulder[1])
    if left_y_diff > TPOSE_ARM_THRESHOLD or right_y_diff > TPOSE_ARM_THRESHOLD:
        return False

    # Check wrists are extended outward beyond shoulders (x axis)
    left_extended  = lshoulder[0] - lwrist[0]   # left wrist should be further left
    right_extended = rwrist[0] - rshoulder[0]    # right wrist should be further right
    if left_extended < TPOSE_EXT_THRESHOLD or right_extended < TPOSE_EXT_THRESHOLD:
        return False

    return True


# ── Calibration ───────────────────────────────────────────────────────────────

async def calibrate_player(ws, player_num, already_claimed, api_token=None):
    """
    Wait for a person to hold a T-pose for TPOSE_HOLD_SECONDS.
    Returns the locked person ID for this player.
    Skips person IDs already claimed by other players.
    """
    print(f"\n  Player {player_num}: Strike a T-pose and hold it for {int(TPOSE_HOLD_SECONDS)} seconds...")
    print(f"  (Arms out straight, level with your shoulders)\n")

    pose_start  = {}   # person_id -> timestamp when T-pose began

    async for message in ws:
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            continue

        now = time.monotonic()
        persons = data.get('persons', [])

        for person in persons:
            pid = person.get('id', 0)
            if pid in already_claimed:
                continue

            kps = extract_keypoints(person)

            if is_tpose(kps):
                if pid not in pose_start:
                    pose_start[pid] = now
                    print(f"  T-pose detected for person {pid} — hold it...")
                elif now - pose_start[pid] >= TPOSE_HOLD_SECONDS:
                    print(f"  Player {player_num} locked to person ID {pid}.")
                    return pid
            else:
                if pid in pose_start:
                    print(f"  Pose broken for person {pid} — try again.")
                    del pose_start[pid]


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

    def _mac(self, player_slot, tracker_id):
        return bytes([0x52, 0x75, 0x56, 0x57, player_slot & 0xFF, tracker_id & 0xFF])

    def send_handshake(self, player_slot, tracker_id):
        mac = self._mac(player_slot, tracker_id)
        firmware = b'ruview-bridge\x00'
        pkt  = struct.pack('>I', PACKET_HANDSHAKE)
        pkt += struct.pack('>Q', self._pnum())
        pkt += struct.pack('>IIIIII', 0, 0, 0, 0, 0, 0)
        pkt += firmware
        pkt += mac
        self.sock.sendto(pkt, (self.host, self.port))

    def send_rotation(self, player_slot, tracker_id, quat):
        x, y, z, w = quat
        pkt  = struct.pack('>I', PACKET_ROTATION_DATA)
        pkt += struct.pack('>Q', self._pnum())
        pkt += struct.pack('B', (player_slot * TRACKER_COUNT) + tracker_id)
        pkt += struct.pack('B', 1)
        pkt += struct.pack('>ffff', x, y, z, w)
        pkt += struct.pack('B', 0)
        self.sock.sendto(pkt, (self.host, self.port))


# ── Bridge loop ───────────────────────────────────────────────────────────────

async def run_bridge(ruview_ws_url, slimevr_host, slimevr_port, num_players, api_token=None):
    sender   = SlimeVRSender(slimevr_host, slimevr_port)
    handshook = set()

    headers = {}
    if api_token:
        headers['Authorization'] = f'Bearer {api_token}'

    print(f"\nConnecting to ruview  → {ruview_ws_url}")
    print(f"Forwarding to SlimeVR → {slimevr_host}:{slimevr_port}")
    print(f"Players: {num_players}\n")

    async with websockets.connect(ruview_ws_url, additional_headers=headers) as ws:
        print("Connected to ruview.\n")
        print("=" * 50)
        print("  T-POSE CALIBRATION")
        print("=" * 50)

        # Calibrate each player in turn
        player_map  = {}   # player_slot (0,1) -> locked person_id
        claimed_ids = set()

        for slot in range(num_players):
            pid = await calibrate_player(ws, slot + 1, claimed_ids, api_token)
            player_map[slot] = pid
            claimed_ids.add(pid)

        print("\n" + "=" * 50)
        print("  CALIBRATION COMPLETE")
        for slot, pid in player_map.items():
            print(f"  Player {slot + 1} → person ID {pid}")
        print("=" * 50)
        print("\nStreaming body tracking data to SlimeVR...\n")

        # Stream tracking data using locked player IDs only
        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            for slot, pid in player_map.items():
                person = next(
                    (p for p in data.get('persons', []) if p.get('id') == pid),
                    None
                )
                if not person:
                    continue

                kps      = extract_keypoints(person)
                trackers = compute_trackers(kps)

                for tid, quat in trackers.items():
                    key = (slot, tid)
                    if key not in handshook:
                        sender.send_handshake(slot, tid)
                        handshook.add(key)
                        await asyncio.sleep(0.05)

                    sender.send_rotation(slot, tid, quat)


def main():
    parser = argparse.ArgumentParser(description='ruview → SlimeVR body tracking bridge with T-pose calibration')
    parser.add_argument(
        '--ruview-ws',
        default='ws://192.168.12.150:3001/ws/sensing',
        help='ruview WebSocket URL',
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
        '--players',
        type=int,
        default=2,
        choices=[1, 2],
        help='Number of players to calibrate (default: 2)',
    )
    parser.add_argument(
        '--api-token',
        default='a7f3d2e1-9b4c-4f8a-b6e2-3d5c1a0f7e94',
        help='ruview API bearer token',
    )
    args = parser.parse_args()

    asyncio.run(run_bridge(
        args.ruview_ws,
        args.slimevr_host,
        args.slimevr_port,
        args.players,
        args.api_token,
    ))


if __name__ == '__main__':
    main()
