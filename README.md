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

- **Registration node** — a marked spot near the entrance. Stand in it for 1.5 seconds and you are automatically assigned a player slot.
- **Exit node** — a marked zone near the exit. Walk through it and your slot is automatically freed for the next player.

This scales to any number of simultaneous players. In a home setup the defaults work out of the box. In a VR farm, players walk in, register, play, walk out, and deregister — the system manages itself.

### Trackers per player
- Hip (torso rotation — eliminates thumbstick turning in VR)
- Chest
- Left + right thigh
- Left + right shin

### Setup — step by step

**Prerequisites:**
- ruview server running on your dedicated machine
- SlimeVR Server installed on your gaming PC (download from [slimevr.dev](https://slimevr.dev))
- SlimeVR companion app sideloaded on both Quest 3 headsets via [SideQuest](https://sidequestvr.com)
- Both headsets on the same local network as the ruview server
- Two marked spots on the floor — one near the entrance (registration), one near the exit

**Steps:**

1. Start SlimeVR Server on your gaming PC
2. Open the SlimeVR companion app on both Quest 3 headsets and connect to the server
3. Start the bridge on the ruview machine:
   ```bash
   python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP
   ```
4. Player 1 walks to the registration spot and stands still for 1.5 seconds — registered automatically
5. Player 2 does the same
6. Both players are live — body tracking streams to SlimeVR immediately
7. Launch your VR game. Hip rotation maps to body turning — no thumbstick needed
8. When done, each player walks through the exit zone to deregister

**Notes:**
- Anyone who does not stand at the registration spot is ignored entirely
- Scales beyond 2 players with `--max-players`
- For SteamVR games, configure hip-to-locomotion turning in SteamVR input bindings per game

### Run command
```bash
# Two players (default)
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP

# VR farm — 10 players, custom zone positions
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP \
    --max-players 10 \
    --reg-zone 0.1,0.5 --reg-radius 0.08 \
    --exit-zone 0.9,0.5 --exit-radius 0.10
```

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
