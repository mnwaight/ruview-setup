# ruview-setup

WiFi-based whole-home presence sensing, biometric monitoring, and VR body tracking.

## Credits & Attribution

This project is a setup package and tooling wrapper. The core technologies are built by others — please go star their repos.

**[RuView by ruvnet](https://github.com/ruvnet/ruview)** — all WiFi sensing, ML inference, ESP32 firmware, and server code. This is the engine that makes everything work.

**[SlimeVR](https://github.com/SlimeVR/SlimeVR-Server)** — the open source body tracking platform that receives our pose data and feeds it into VR games. The SlimeVR UDP protocol is what the bridge speaks to deliver virtual trackers to SteamVR.

**Original code in this repo:**
- `install.sh` — cross-platform auto-detecting installer (WSL2, native Linux, macOS)
- `ruview-slimevr-bridge.py` — SlimeVR VR body tracking bridge with T-pose player calibration
- `flash_board.sh` — WSL2 board flashing helper

**From ruvnet/ruview:**
- `esp32-csi-node.bin` and all firmware binaries — ESP32-S3 CSI capture firmware
- `provision.py` — ESP32 WiFi provisioning script
- The ruview Docker image (`ruvnet/wifi-densepose`) — all sensing, ML inference, and API

## What this does

- Detects presence, movement, and vital signs (breathing + heart rate) through walls using WiFi CSI signals
- Runs entirely on your local network — no cloud, no external connections
- Feeds real-time skeletal pose data into SlimeVR for full body tracking in VR (Quest 3, etc.)
- Supports up to 9+ sensor nodes across multiple floors

## Hardware required

- ESP32-S3 development boards (~$6-9 each, 1 per 400-500 sq ft)
- USB-C cables (one per board)
- 5V USB wall adapters (one per board)
- A Linux/macOS machine to run the server (old desktop works fine)

## Quick start

```bash
git clone https://github.com/mnwaight/ruview-setup.git
cd ruview-setup
chmod +x install.sh
./install.sh
```

The installer auto-detects your OS, distro, and environment (WSL2, native Linux, macOS) and handles everything.

## What the installer does

1. Installs Python, esptool, Docker, and all dependencies
2. Sets up serial port permissions and ESP32 udev rules
3. Pulls and starts the ruview Docker server with auth and no external connections
4. Walks you through flashing each ESP32-S3 board
5. Launches the SlimeVR body tracking bridge

## VR body tracking (experimental)

> **Note:** This is a prototype integration. Standard SlimeVR hardware uses ESP32 boards physically strapped to the body as IMU sensors. What we are doing here is fundamentally different — spoofing that protocol using WiFi-detected skeletal pose data from a room sensor. It may not behave identically to physical SlimeVR trackers and results will vary by game and platform.
>
> The ruview pose data stream opens up a lot of possibilities beyond this prototype. Other integration targets worth exploring include OSC (supported natively by VRChat), Virtual Motion Capture for SteamVR, and direct game SDK integrations. This bridge is a starting point, not a finished product.

The included `ruview-slimevr-bridge.py` reads ruview's 17-keypoint pose data and attempts to forward it to SlimeVR server as virtual trackers. Supports two simultaneous players with T-pose calibration to prevent bystanders from interfering with tracking.

### Zone-based registration

Registration is handled by physical floor zones — no T-poses, no menus, no staff intervention. Two marked spots on the floor do all the work.

- **Registration zone** — a marked spot near the entrance. Stand in it for 1.5 seconds to claim a player slot.
- **Exit zone** — a marked zone near the exit. Walk through it to release your slot.

This scales to any number of simultaneous players. At home it works identically — one person sets up zones once, then anyone with a headset just walks in and plays. In a VR farm, players walk in, register, play, walk out, and deregister with no staff intervention.

### Trackers per player
- Hip (torso rotation — eliminates thumbstick turning in VR)
- Chest
- Left + right thigh
- Left + right shin

### Re-registration and ID loss protection

If ruview temporarily loses track of a player and assigns them a new person ID, the bridge automatically reacquires them by matching their new position to their last known position. Players do not need to re-register.

If a registered player walks near the registration zone during play, nothing happens — only unregistered person IDs can trigger registration. A 5-second cooldown also prevents the zone from immediately re-registering someone who just deregistered.

---

### Administrator setup — first time only

Before players can use the system, an administrator sets the physical zone locations once. This is done by standing at each spot so the system can record the coordinates.

**Make sure you are the only person in the room during setup.**

```bash
python3 ruview-slimevr-bridge.py --setup
```

The setup wizard will:
1. Ask you to stand at the **registration spot** and hold still for 3 seconds
2. Ask you to stand at the **exit spot** and hold still for 3 seconds
3. Save both coordinates to `zones.json` in the same directory

After setup, mark both spots on the floor with tape, a mat, or a sign. Zone config persists across restarts — you only need to run setup once unless you rearrange the room.

---

### Player setup — step by step

**Prerequisites:**
- ruview server running on your dedicated machine
- SlimeVR Server installed on your gaming PC (download from [slimevr.dev](https://slimevr.dev))
- SlimeVR companion app sideloaded on both Quest 3 headsets via [SideQuest](https://sidequestvr.com)
- Both headsets on the same local network as the ruview server
- Admin has completed zone setup and floor spots are marked

**Steps:**

1. Start SlimeVR Server on your gaming PC
2. Open the SlimeVR companion app on both Quest 3 headsets and connect to the server
3. Start the bridge on the ruview machine:
   ```bash
   python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP
   ```
4. Each player walks to the registration spot and stands still for 1.5 seconds — registered automatically
5. Body tracking streams to SlimeVR immediately after registration
6. Launch your VR game — hip rotation maps to body turning, no thumbstick needed
7. When done, each player walks through the exit zone to deregister

**Scaling:** additional players with headsets just walk to the registration spot. No config changes needed. Set `--max-players` to match your space.

### Run command
```bash
# First time zone setup (admin only)
python3 ruview-slimevr-bridge.py --setup

# Two players (default)
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP

# VR farm — 10 players
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP --max-players 10
```

## Future possibilities

The ruview pose stream tracks every person in the space — registered players and everyone else. This opens up two compelling directions beyond the current prototype:

**Non-VR users as in-game NPCs**
Unregistered people in the room already have their full skeletal pose tracked. A game engine integration (Unity or Unreal plugin) consuming the unregistered pose stream could spawn and animate NPC characters driven by real human movement. Spectators watching from the sidelines appear as crowd NPCs. An instructor walks the real space and appears as a guide character inside the game. No wearables, no cameras — just the WiFi mesh that is already there.

**VR players seeing real people through walls**
The inverse direction — VR players could see unregistered room occupants rendered as outlines or ghost markers inside their headset, showing where real people are in the physical space in real time. This is both a safety feature (avoid collisions with non-VR people) and a potential game mechanic. A haunted house experience where a performer moving through the real room appears as a ghost that VR players can actually see and react to.

Both of these are natural extensions of the existing pose data pipeline. The bridge already separates registered players from unregistered persons — the unregistered stream just needs a consumer.

## Files

| File | Purpose |
|------|---------|
| `install.sh` | Auto-detecting setup installer |
| `ruview-slimevr-bridge.py` | ruview → SlimeVR UDP bridge |
| `flash_board.sh` | Single board flash helper (WSL2) |
| `provision.py` | ESP32 WiFi provisioning |
| `bootloader.bin` | ESP32-S3 firmware |
| `partition-table.bin` | ESP32-S3 firmware |
| `ota_data_initial.bin` | ESP32-S3 firmware |
| `esp32-csi-node.bin` | ESP32-S3 firmware |

## Node placement guide

- 1 node per 400-500 sq ft for presence detection
- 1 node per room for biometric accuracy
- Place pairs at opposite corners of each floor for best coverage
- Basement/open spaces: 1 node centered is sufficient
