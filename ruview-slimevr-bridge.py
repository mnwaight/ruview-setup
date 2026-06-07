#!/usr/bin/env python3
"""
ruview-slimevr-bridge.py
Bridges ruview WiFi pose data to SlimeVR virtual trackers.

Reads 17-keypoint COCO pose from ruview WebSocket and sends
quaternion rotation packets to SlimeVR server via UDP.

Registration is zone-based — a marked spot on the floor acts as
a registration node. When a person enters that zone they are
automatically assigned a player slot. When they cross the exit
zone they are deregistered. No T-pose or headset communication needed.

Scales to any number of simultaneous players (home or VR farm).

Usage:
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.102
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.102 --max-players 10
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.102 \\
      --reg-zone 0.1,0.5 --reg-radius 0.08 \\
      --exit-zone 0.9,0.5 --exit-radius 0.1
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

# Default zone centers in normalized ruview space (0.0 - 1.0)
# Registration node: left side of room
DEFAULT_REG_ZONE   = (0.1, 0.5)
DEFAULT_REG_RADIUS = 0.08

# Exit/deregistration zone: right side of room
DEFAULT_EXIT_ZONE   = (0.9, 0.5)
DEFAULT_EXIT_RADIUS = 0.10

# How long a person must stand in the registration zone before locking
REG_DWELL_SECONDS = 1.5


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

def distance_2d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def in_zone(position_xy, zone_center, zone_radius):
    return distance_2d(position_xy, zone_center) <= zone_radius

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

def person_position_2d(kps):
    """Estimate person's floor position from hip midpoint."""
    lhip = kps.get('left_hip')
    rhip = kps.get('right_hip')
    if lhip and rhip:
        mid = midpoint(lhip, rhip)
        return (mid[0], mid[2])   # x and depth as floor coords
    return None

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


# ── Zone-based session registry ───────────────────────────────────────────────

class SessionRegistry:
    def __init__(self, reg_zone, reg_radius, exit_zone, exit_radius, max_players):
        self.reg_zone    = reg_zone
        self.reg_radius  = reg_radius
        self.exit_zone   = exit_zone
        self.exit_radius = exit_radius
        self.max_players = max_players

        self.pid_to_slot  = {}   # person_id -> player_slot
        self.slot_to_pid  = {}   # player_slot -> person_id
        self.next_slot    = 0

        # Tracks how long each unregistered person has been in the reg zone
        self.reg_dwell    = {}   # person_id -> entry timestamp

    def _next_available_slot(self):
        used = set(self.slot_to_pid.keys())
        for s in range(self.max_players):
            if s not in used:
                return s
        return None

    def process(self, persons):
        """
        Update registry based on current frame's person list.
        Returns (newly_registered, newly_deregistered) person_id sets.
        """
        now = time.monotonic()
        current_pids = {p.get('id') for p in persons}
        registered   = set()
        deregistered = set()

        for person in persons:
            pid = person.get('id')
            kps = extract_keypoints(person)
            pos = person_position_2d(kps)
            if pos is None:
                continue

            # Deregistration check — registered player enters exit zone
            if pid in self.pid_to_slot:
                if in_zone(pos, self.exit_zone, self.exit_radius):
                    slot = self.pid_to_slot.pop(pid)
                    del self.slot_to_pid[slot]
                    self.reg_dwell.pop(pid, None)
                    deregistered.add(pid)
                    print(f"  Player {slot + 1} (person {pid}) deregistered at exit zone.")

            # Registration check — unregistered person enters reg zone
            elif len(self.slot_to_pid) < self.max_players:
                if in_zone(pos, self.reg_zone, self.reg_radius):
                    if pid not in self.reg_dwell:
                        self.reg_dwell[pid] = now
                        print(f"  Person {pid} entering registration zone — hold position...")
                    elif now - self.reg_dwell[pid] >= REG_DWELL_SECONDS:
                        slot = self._next_available_slot()
                        if slot is not None:
                            self.pid_to_slot[pid] = slot
                            self.slot_to_pid[slot] = pid
                            self.reg_dwell.pop(pid, None)
                            registered.add(pid)
                            print(f"  Player {slot + 1} registered (person {pid}).")
                else:
                    self.reg_dwell.pop(pid, None)

        # Clean up dwell tracking for people who left the frame
        gone = set(self.reg_dwell.keys()) - current_pids
        for pid in gone:
            del self.reg_dwell[pid]

        return registered, deregistered

    def active_players(self):
        return dict(self.slot_to_pid)   # slot -> pid


# ── Bridge loop ───────────────────────────────────────────────────────────────

async def run_bridge(ruview_ws_url, slimevr_host, slimevr_port,
                     reg_zone, reg_radius, exit_zone, exit_radius,
                     max_players, api_token=None):

    sender   = SlimeVRSender(slimevr_host, slimevr_port)
    registry = SessionRegistry(reg_zone, reg_radius, exit_zone, exit_radius, max_players)
    handshook = set()

    headers = {}
    if api_token:
        headers['Authorization'] = f'Bearer {api_token}'

    print(f"\nConnecting to ruview  → {ruview_ws_url}")
    print(f"Forwarding to SlimeVR → {slimevr_host}:{slimevr_port}")
    print(f"Max players: {max_players}")
    print(f"Registration zone: center={reg_zone} radius={reg_radius}")
    print(f"Exit zone:         center={exit_zone} radius={exit_radius}")
    print(f"\nWaiting for players to enter the registration zone...\n")

    async with websockets.connect(ruview_ws_url, additional_headers=headers) as ws:
        print("Connected to ruview.\n")

        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            persons = data.get('persons', [])
            registry.process(persons)

            for slot, pid in registry.active_players().items():
                person = next(
                    (p for p in persons if p.get('id') == pid),
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


def parse_xy(s):
    x, y = s.split(',')
    return (float(x), float(y))


def main():
    parser = argparse.ArgumentParser(
        description='ruview → SlimeVR bridge with zone-based player registration'
    )
    parser.add_argument('--ruview-ws', default='ws://192.168.12.150:3001/ws/sensing')
    parser.add_argument('--slimevr-host', required=True)
    parser.add_argument('--slimevr-port', type=int, default=6969)
    parser.add_argument('--max-players', type=int, default=2,
                        help='Maximum simultaneous players (default 2, no upper limit)')
    parser.add_argument('--reg-zone', type=parse_xy,
                        default=DEFAULT_REG_ZONE,
                        metavar='X,Y',
                        help='Registration zone center in normalized coords (default: 0.1,0.5)')
    parser.add_argument('--reg-radius', type=float, default=DEFAULT_REG_RADIUS,
                        help='Registration zone radius (default: 0.08)')
    parser.add_argument('--exit-zone', type=parse_xy,
                        default=DEFAULT_EXIT_ZONE,
                        metavar='X,Y',
                        help='Exit/deregistration zone center (default: 0.9,0.5)')
    parser.add_argument('--exit-radius', type=float, default=DEFAULT_EXIT_RADIUS,
                        help='Exit zone radius (default: 0.10)')
    parser.add_argument('--api-token', default='a7f3d2e1-9b4c-4f8a-b6e2-3d5c1a0f7e94')
    args = parser.parse_args()

    asyncio.run(run_bridge(
        args.ruview_ws,
        args.slimevr_host,
        args.slimevr_port,
        args.reg_zone,
        args.reg_radius,
        args.exit_zone,
        args.exit_radius,
        args.max_players,
        args.api_token,
    ))


if __name__ == '__main__':
    main()
