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

## VR body tracking (SlimeVR bridge)

The included `ruview-slimevr-bridge.py` reads ruview's 17-keypoint pose data and sends it to SlimeVR server as virtual trackers. Supports two simultaneous players with T-pose calibration to prevent bystanders from interfering with tracking.

### T-pose calibration

Before the session starts, each player strikes a T-pose (arms out straight, level with shoulders) and holds it for 2 seconds. The bridge locks that person ID to that player slot. Only the two calibrated players are ever forwarded to SlimeVR — anyone else who walks into the room is ignored.

### Trackers per player
- Hip (torso rotation — eliminates thumbstick turning in VR)
- Chest
- Left + right thigh
- Left + right shin

### Two player setup — step by step

**Prerequisites:**
- ruview server running on your dedicated machine
- SlimeVR Server installed on your gaming PC (download from [slimevr.dev](https://slimevr.dev))
- SlimeVR companion app installed on both Quest 3 headsets from the Meta store
- Both headsets on the same local network as the ruview server

**Steps:**

1. Start SlimeVR Server on your gaming PC
2. Open the SlimeVR companion app on both Quest 3 headsets and connect them to the server
3. On the machine running ruview, start the bridge:
   ```bash
   python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP
   ```
4. The bridge will prompt Player 1 to calibrate first
5. Player 1 stands in the play space and strikes a T-pose — arms out straight, level with shoulders — and holds it for 2 seconds
6. The bridge confirms Player 1 is locked, then prompts Player 2
7. Player 2 strikes their T-pose and holds for 2 seconds
8. Both players are now locked — body tracking begins automatically
9. Launch your VR game. Hip rotation maps to body turning — no thumbstick needed

**Notes:**
- Anyone else who walks into the room during a session is ignored
- If tracking feels off, restart the bridge and recalibrate
- For SteamVR games, configure hip-to-locomotion turning in the SteamVR input bindings per game

### Run command
```bash
# Two players (default)
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP

# Single player
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP --players 1
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
