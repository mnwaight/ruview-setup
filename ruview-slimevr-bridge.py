#!/usr/bin/env python3
"""
ruview-slimevr-bridge.py
Bridges ruview WiFi pose data to SlimeVR virtual trackers.

Reads 17-keypoint COCO pose from ruview WebSocket and sends
quaternion rotation packets to SlimeVR server via UDP.

Zone-based registration — physical floor markers handle everything:
  - Stand at the registration spot for 1.5s to claim a player slot
  - Walk through the exit zone to release your slot
  - Scales to any number of simultaneous players

First time setup:
  python3 ruview-slimevr-bridge.py --setup

Normal use:
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.102
  python3 ruview-slimevr-bridge.py --slimevr-host 192.168.12.102 --max-players 10
"""

import asyncio
import json
import math
import socket
import struct
import argparse
import time
import os
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

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'zones.json')

# Defaults used if no zones.json exists yet
DEFAULT_REG_ZONE    = (0.1, 0.5)
DEFAULT_REG_RADIUS  = 0.08
DEFAULT_EXIT_ZONE   = (0.9, 0.5)
DEFAULT_EXIT_RADIUS = 0.10

REG_DWELL_SECONDS    = 1.5   # seconds to stand in reg zone before locking
SETUP_DWELL_SECONDS  = 3.0   # seconds to stand still during zone setup
REACQUIRE_DISTANCE   = 0.15  # max normalized distance for ID reacquisition
COOLDOWN_SECONDS     = 5.0   # seconds a zone position is locked out after deregistration


# ── Config persistence ────────────────────────────────────────────────────────

def load_zones():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            c = json.load(f)
        print(f"Loaded zone config from {CONFIG_FILE}")
        return (
            tuple(c['reg_zone']),
            c['reg_radius'],
            tuple(c['exit_zone']),
            c['exit_radius'],
        )
    print("No zones.json found — using defaults. Run --setup to configure zones.")
    return DEFAULT_REG_ZONE, DEFAULT_REG_RADIUS, DEFAULT_EXIT_ZONE, DEFAULT_EXIT_RADIUS

def save_zones(reg_zone, reg_radius, exit_zone, exit_radius):
    config = {
        'reg_zone':    list(reg_zone),
        'reg_radius':  reg_radius,
        'exit_zone':   list(exit_zone),
        'exit_radius': exit_radius,
    }
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"Zone config saved to {CONFIG_FILE}")


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
    lhip = kps.get('left_hip')
    rhip = kps.get('right_hip')
    if lhip and rhip:
        mid = midpoint(lhip, rhip)
        return (mid[0], mid[2])
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


# ── Interactive zone setup ────────────────────────────────────────────────────

async def run_setup(ruview_ws_url, api_token=None):
    """
    Interactive zone setup mode. Administrator stands at each zone location
    so the system can record the coordinates. Saves to zones.json.
    """
    headers = {}
    if api_token:
        headers['Authorization'] = f'Bearer {api_token}'

    print("\n" + "=" * 55)
    print("  ZONE SETUP MODE")
    print("=" * 55)
    print("""
This will record the positions of two floor zones:

  1. REGISTRATION zone — where players stand to join
  2. EXIT zone — where players walk to leave the session

You will stand at each location so the system can detect
your position. Make sure you are the only person in the
room during setup, or the only one moving.
""")

    async with websockets.connect(ruview_ws_url, additional_headers=headers) as ws:

        async def capture_position(zone_name):
            print(f"─── {zone_name} ───")
            print(f"Stand at the {zone_name.lower()} spot now and hold still...")
            positions = []
            start = None

            async for message in ws:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                persons = data.get('persons', [])
                if not persons:
                    continue

                person = persons[0]
                kps = extract_keypoints(person)
                pos = person_position_2d(kps)
                if pos is None:
                    continue

                if start is None:
                    start = time.monotonic()

                elapsed = time.monotonic() - start
                positions.append(pos)

                remaining = SETUP_DWELL_SECONDS - elapsed
                print(f"\r  Detected at ({pos[0]:.3f}, {pos[1]:.3f}) — "
                      f"hold for {max(0, remaining):.1f}s...   ", end='', flush=True)

                if elapsed >= SETUP_DWELL_SECONDS:
                    avg_x = sum(p[0] for p in positions) / len(positions)
                    avg_y = sum(p[1] for p in positions) / len(positions)
                    print(f"\n  {zone_name} set to ({avg_x:.4f}, {avg_y:.4f})\n")
                    return (avg_x, avg_y)

        reg_zone  = await capture_position("REGISTRATION ZONE")
        exit_zone = await capture_position("EXIT ZONE")

        reg_radius  = DEFAULT_REG_RADIUS
        exit_radius = DEFAULT_EXIT_RADIUS

        save_zones(reg_zone, reg_radius, exit_zone, exit_radius)

        print("\n" + "=" * 55)
        print("  SETUP COMPLETE")
        print("=" * 55)
        print(f"""
  Registration zone : {reg_zone}  radius: {reg_radius}
  Exit zone         : {exit_zone}  radius: {exit_radius}

  Mark these spots on the floor (tape, mat, or sign).
  Players stand at the registration spot to join.
  Players walk through the exit spot to leave.

  Start the bridge normally when ready:
    python3 ruview-slimevr-bridge.py --slimevr-host YOUR_PC_IP
""")


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

        self.pid_to_slot      = {}   # person_id -> player_slot
        self.slot_to_pid      = {}   # player_slot -> person_id
        self.slot_last_pos    = {}   # player_slot -> last known position
        self.reg_dwell        = {}   # person_id -> entry timestamp
        self.exit_cooldowns   = []   # list of (timestamp, position) — recently freed zones

    def _next_available_slot(self):
        used = set(self.slot_to_pid.keys())
        for s in range(self.max_players):
            if s not in used:
                return s
        return None

    def _in_cooldown(self, pos):
        now = time.monotonic()
        self.exit_cooldowns = [
            (t, p) for t, p in self.exit_cooldowns
            if now - t < COOLDOWN_SECONDS
        ]
        return any(distance_2d(pos, p) < self.reg_radius for _, p in self.exit_cooldowns)

    def _try_reacquire(self, pid, pos):
        """
        If ruview lost and reassigned a player's ID, find their slot
        by nearest last-known position and transfer the assignment.
        """
        best_slot = None
        best_dist = REACQUIRE_DISTANCE

        for slot, last_pos in self.slot_last_pos.items():
            if slot in self.slot_to_pid:
                continue   # slot already has an active pid
            d = distance_2d(pos, last_pos)
            if d < best_dist:
                best_dist = d
                best_slot = slot

        if best_slot is not None:
            self.pid_to_slot[pid] = best_slot
            self.slot_to_pid[best_slot] = pid
            print(f"  Reacquired: person {pid} matched to player slot {best_slot + 1}.")
            return True
        return False

    def process(self, persons):
        now = time.monotonic()
        current_pids = {p.get('id') for p in persons}

        # Detect slots whose pid disappeared — mark slot as vacant but keep last_pos
        for slot, pid in list(self.slot_to_pid.items()):
            if pid not in current_pids:
                del self.pid_to_slot[pid]
                del self.slot_to_pid[slot]

        for person in persons:
            pid = person.get('id')
            kps = extract_keypoints(person)
            pos = person_position_2d(kps)
            if pos is None:
                continue

            # Update last known position for active players
            if pid in self.pid_to_slot:
                self.slot_last_pos[self.pid_to_slot[pid]] = pos

                # Deregistration check
                if in_zone(pos, self.exit_zone, self.exit_radius):
                    slot = self.pid_to_slot.pop(pid)
                    del self.slot_to_pid[slot]
                    self.exit_cooldowns.append((now, pos))
                    self.reg_dwell.pop(pid, None)
                    print(f"  Player {slot + 1} (person {pid}) deregistered at exit zone.")
                continue

            # Try reacquiring a slot for a new pid near a recently lost player
            if self._try_reacquire(pid, pos):
                continue

            # Registration check for new unregistered persons
            if len(self.slot_to_pid) >= self.max_players:
                continue

            if in_zone(pos, self.reg_zone, self.reg_radius):
                # Don't register if this position was just freed (cooldown)
                if self._in_cooldown(pos):
                    continue

                if pid not in self.reg_dwell:
                    self.reg_dwell[pid] = now
                    print(f"  Person {pid} in registration zone — hold for "
                          f"{REG_DWELL_SECONDS:.0f}s...")
                elif now - self.reg_dwell[pid] >= REG_DWELL_SECONDS:
                    slot = self._next_available_slot()
                    if slot is not None:
                        self.pid_to_slot[pid] = slot
                        self.slot_to_pid[slot] = pid
                        self.slot_last_pos[slot] = pos
                        self.reg_dwell.pop(pid, None)
                        print(f"  Player {slot + 1} registered (person {pid}).")
            else:
                self.reg_dwell.pop(pid, None)

        # Clean up dwell tracking for people who left the frame
        for pid in list(self.reg_dwell):
            if pid not in current_pids:
                del self.reg_dwell[pid]

    def active_players(self):
        return dict(self.slot_to_pid)


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
    print(f"Max players    : {max_players}")
    print(f"Register zone  : center={reg_zone}  radius={reg_radius}")
    print(f"Exit zone      : center={exit_zone}  radius={exit_radius}")
    print(f"\nReady. Players stand at the registration spot to join.\n")

    reconnect_delay = 2
    while True:
        try:
            async with websockets.connect(ruview_ws_url, additional_headers=headers) as ws:
                print("Connected to ruview.\n")
                reconnect_delay = 2

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

        except (websockets.exceptions.ConnectionClosed,
                websockets.exceptions.WebSocketException,
                OSError) as e:
            print(f"ruview disconnected ({e}). Reconnecting in {reconnect_delay}s...")
            handshook.clear()
            await asyncio.sleep(reconnect_delay)
            reconnect_delay = min(reconnect_delay * 2, 30)


def parse_xy(s):
    x, y = s.split(',')
    return (float(x), float(y))


def main():
    parser = argparse.ArgumentParser(
        description='ruview → SlimeVR bridge with zone-based player registration'
    )
    parser.add_argument('--setup', action='store_true',
                        help='Run interactive zone setup mode (admin, one time)')
    parser.add_argument('--ruview-ws', default='ws://localhost:3001/ws/sensing')
    parser.add_argument('--slimevr-host', default=None)
    parser.add_argument('--slimevr-port', type=int, default=6969)
    parser.add_argument('--max-players', type=int, default=2)
    parser.add_argument('--reg-zone', type=parse_xy, default=None, metavar='X,Y')
    parser.add_argument('--reg-radius', type=float, default=None)
    parser.add_argument('--exit-zone', type=parse_xy, default=None, metavar='X,Y')
    parser.add_argument('--exit-radius', type=float, default=None)
    parser.add_argument('--api-token', default=None)
    args = parser.parse_args()

    if args.setup:
        asyncio.run(run_setup(args.ruview_ws, args.api_token))
        return

    if not args.slimevr_host:
        parser.error('--slimevr-host is required unless running --setup')

    # Load saved zones, then allow CLI overrides
    reg_zone, reg_radius, exit_zone, exit_radius = load_zones()
    if args.reg_zone:   reg_zone    = args.reg_zone
    if args.reg_radius: reg_radius  = args.reg_radius
    if args.exit_zone:  exit_zone   = args.exit_zone
    if args.exit_radius: exit_radius = args.exit_radius

    asyncio.run(run_bridge(
        args.ruview_ws,
        args.slimevr_host,
        args.slimevr_port,
        reg_zone,
        reg_radius,
        exit_zone,
        exit_radius,
        args.max_players,
        args.api_token,
    ))


if __name__ == '__main__':
    main()
