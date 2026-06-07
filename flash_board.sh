#!/bin/bash
# Flash and provision a single ESP32-S3 ruview node.
# Usage: ./flash_board.sh <COM_PORT> <BOARD_NUMBER> <WIFI_SSID> <WIFI_PASSWORD> <SERVER_IP>
# Example: ./flash_board.sh COM3 1 "MyWiFi" "mypassword" 192.168.1.100
#
# In WSL2, Windows COM3 = /dev/ttyS3, COM9 = /dev/ttyS9, etc.

set -e

COM_PORT="${1}"
BOARD_NUM="${2}"
SSID="${3}"
WIFI_PASS="${4}"
SERVER_IP="${5}"

if [ -z "$COM_PORT" ] || [ -z "$BOARD_NUM" ] || [ -z "$SSID" ] || [ -z "$WIFI_PASS" ] || [ -z "$SERVER_IP" ]; then
  echo "Usage: $0 <COM_PORT> <BOARD_NUMBER> <WIFI_SSID> <WIFI_PASSWORD> <SERVER_IP>"
  echo "Example: $0 COM3 1 MyWiFi secret 192.168.1.100"
  exit 1
fi

# Convert Windows COM port to WSL2 serial device (COM3 -> /dev/ttyS3)
if [[ "$COM_PORT" == COM* ]]; then
  NUM="${COM_PORT#COM}"
  PORT="/dev/ttyS${NUM}"
else
  PORT="$COM_PORT"
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Flashing board $BOARD_NUM on $PORT ==="
python3 -m esptool --chip esp32s3 --port "$PORT" --baud 460800 \
  write_flash \
  0x0     "$SCRIPT_DIR/bootloader.bin" \
  0x8000  "$SCRIPT_DIR/partition-table.bin" \
  0xf000  "$SCRIPT_DIR/ota_data_initial.bin" \
  0x20000 "$SCRIPT_DIR/esp32-csi-node.bin"

echo ""
echo "=== Provisioning board $BOARD_NUM (node-id: $BOARD_NUM) ==="
python3 "$SCRIPT_DIR/provision.py" \
  --port "$PORT" \
  --ssid "$SSID" \
  --password "$WIFI_PASS" \
  --target-ip "$SERVER_IP" \
  --node-id "$BOARD_NUM"

echo ""
echo "=== Board $BOARD_NUM done! Unplug and label it. ==="
