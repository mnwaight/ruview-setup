# ruview-setup

WiFi-based whole-home presence sensing, biometric monitoring, and VR body tracking.

## Credits & Attribution

This project is a setup package and tooling wrapper built on top of **[RuView by ruvnet](https://github.com/ruvnet/ruview)** — all sensing, ML, and server code is their work. All credit for the core WiFi-DensePose technology goes to the ruvnet team.

**Original code in this repo:**
- `install.sh` — cross-platform auto-detecting installer
- `ruview-slimevr-bridge.py` — SlimeVR VR body tracking bridge (novel integration)
- `flash_board.sh` — WSL2 board flashing helper

**From ruvnet/ruview:**
- `esp32-csi-node.bin` and all firmware binaries — ESP32-S3 CSI capture firmware
- `provision.py` — ESP32 WiFi provisioning script
- The ruview Docker image (`ruvnet/wifi-densepose`) — all sensing, ML inference, and API

If you find this useful, go star the upstream repo: https://github.com/ruvnet/ruview

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

The included `ruview-slimevr-bridge.py` reads ruview's 17-keypoint pose data and sends it to SlimeVR server as virtual trackers. Provides hip, chest, thigh, and shin tracking for two simultaneous players.

Trackers per person:
- Hip (torso rotation — eliminates thumbstick turning)
- Chest
- Left + right thigh
- Left + right shin

Run command:
```bash
python3 ruview-slimevr-bridge.py --slimevr-host YOUR_GAMING_PC_IP
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
