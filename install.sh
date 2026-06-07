#!/bin/bash
# ruview install script
# Auto-detects OS, environment, and package manager.
# Sets up dependencies, serial permissions, and all ruview tooling.

set -e

RUVIEW_SERVER="192.168.12.150"
RUVIEW_WS="ws://${RUVIEW_SERVER}:3001/ws/sensing"
SLIMEVR_PORT=6969
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info()    { echo -e "${GREEN}[ruview]${NC} $1"; }
warn()    { echo -e "${YELLOW}[ruview]${NC} $1"; }
error()   { echo -e "${RED}[ruview]${NC} $1"; exit 1; }
section() { echo -e "\n${BLUE}── $1 ──${NC}"; }


# ── Environment detection ─────────────────────────────────────────────────────

detect_env() {
    ENV_WSL=false
    ENV_MACOS=false
    ENV_LINUX=false

    if [[ "$(uname)" == "Darwin" ]]; then
        ENV_MACOS=true
        info "Detected: macOS"
    elif grep -qi microsoft /proc/version 2>/dev/null; then
        ENV_WSL=true
        info "Detected: WSL2 (Windows Subsystem for Linux)"
    else
        ENV_LINUX=true
        info "Detected: Native Linux"
    fi
}

detect_distro() {
    DISTRO=""
    PKG_MANAGER=""

    if $ENV_MACOS; then
        DISTRO="macos"
        PKG_MANAGER="brew"
        return
    fi

    if [ -f /etc/os-release ]; then
        source /etc/os-release
        DISTRO_ID="${ID,,}"
    fi

    case "$DISTRO_ID" in
        ubuntu|debian|linuxmint|pop)
            DISTRO="debian"
            PKG_MANAGER="apt"
            ;;
        fedora|rhel|centos|rocky|alma)
            DISTRO="fedora"
            PKG_MANAGER="dnf"
            ;;
        arch|manjaro|endeavouros)
            DISTRO="arch"
            PKG_MANAGER="pacman"
            ;;
        opensuse*|sles)
            DISTRO="suse"
            PKG_MANAGER="zypper"
            ;;
        *)
            warn "Unknown distro: $DISTRO_ID — attempting apt fallback"
            DISTRO="debian"
            PKG_MANAGER="apt"
            ;;
    esac

    info "Distro: $DISTRO_ID, Package manager: $PKG_MANAGER"
}

detect_serial_group() {
    # Fedora/Arch use 'uucp', Debian/Ubuntu use 'dialout'
    if getent group dialout &>/dev/null; then
        SERIAL_GROUP="dialout"
    elif getent group uucp &>/dev/null; then
        SERIAL_GROUP="uucp"
    else
        SERIAL_GROUP="dialout"
    fi
    info "Serial group: $SERIAL_GROUP"
}

detect_serial_port() {
    # Returns the likely serial port path for the connected ESP32
    if $ENV_WSL; then
        # WSL2: scan for ttyS* that correspond to Windows COM ports
        SERIAL_PORT=""
        for port in /dev/ttyS{1..20}; do
            if [ -e "$port" ]; then
                SERIAL_PORT="$port"
                break
            fi
        done
        if [ -z "$SERIAL_PORT" ]; then
            warn "No serial port found. Plug in the ESP32 board and check Device Manager for the COM port."
            warn "Then use: /dev/ttySX where X is the COM port number (e.g. COM3 = /dev/ttyS3)"
        fi
    else
        # Native Linux/macOS: look for USB serial adapters
        for candidate in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1 /dev/cu.usbserial* /dev/cu.SLAB*; do
            if [ -e "$candidate" ]; then
                SERIAL_PORT="$candidate"
                info "Found serial port: $SERIAL_PORT"
                return
            fi
        done
        SERIAL_PORT=""
        warn "No ESP32 detected on USB. Plug in a board and re-run, or specify port manually."
    fi
}


# ── Dependency installation ───────────────────────────────────────────────────

install_python_deps() {
    info "Installing Python dependencies..."

    case "$PKG_MANAGER" in
        apt)
            sudo apt-get update -qq
            sudo apt-get install -y python3 python3-pip curl
            ;;
        dnf)
            sudo dnf install -y python3 python3-pip curl
            ;;
        pacman)
            sudo pacman -Sy --noconfirm python python-pip curl
            ;;
        zypper)
            sudo zypper install -y python3 python3-pip curl
            ;;
        brew)
            brew install python3 curl
            ;;
    esac

    pip3 install esptool websockets --break-system-packages 2>/dev/null \
        || pip3 install esptool websockets
    info "Python dependencies installed."
}

install_docker() {
    if command -v docker &>/dev/null; then
        info "Docker already installed: $(docker --version)"
        return
    fi

    warn "Docker not found. Installing..."
    case "$PKG_MANAGER" in
        apt)
            curl -fsSL https://get.docker.com | sudo sh
            sudo usermod -aG docker "$USER"
            ;;
        dnf)
            sudo dnf install -y docker
            sudo systemctl enable --now docker
            sudo usermod -aG docker "$USER"
            ;;
        pacman)
            sudo pacman -Sy --noconfirm docker
            sudo systemctl enable --now docker
            sudo usermod -aG docker "$USER"
            ;;
        brew)
            warn "Install Docker Desktop for Mac from https://docker.com/products/docker-desktop"
            ;;
    esac
    info "Docker installed. You may need to log out and back in for group permissions."
}

setup_serial_permissions() {
    if $ENV_MACOS; then
        info "macOS: no serial group setup needed."
        return
    fi

    if groups "$USER" | grep -qw "$SERIAL_GROUP"; then
        info "User already in $SERIAL_GROUP group."
    else
        info "Adding $USER to $SERIAL_GROUP group for serial port access..."
        sudo usermod -aG "$SERIAL_GROUP" "$USER"
        warn "Group change requires logout/login to take effect. For this session run: newgrp $SERIAL_GROUP"
    fi

    # Install udev rule for ESP32 if not present
    UDEV_RULE='/etc/udev/rules.d/99-esp32.rules'
    if [ ! -f "$UDEV_RULE" ]; then
        info "Installing ESP32 udev rule..."
        echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="'"$SERIAL_GROUP"'"' \
            | sudo tee "$UDEV_RULE" > /dev/null
        echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666", GROUP="'"$SERIAL_GROUP"'"' \
            | sudo tee -a "$UDEV_RULE" > /dev/null
        sudo udevadm control --reload-rules 2>/dev/null || true
        info "udev rules installed."
    fi
}


# ── ruview server setup ───────────────────────────────────────────────────────

setup_ruview_server() {
    section "ruview Server Setup"

    if ! command -v docker &>/dev/null; then
        error "Docker is required. Run install again or install Docker manually."
    fi

    # Stop existing container if running
    if docker ps -a --format '{{.Names}}' | grep -q '^ruview$'; then
        info "Removing existing ruview container..."
        docker stop ruview 2>/dev/null || true
        docker rm ruview 2>/dev/null || true
    fi

    read -rp "Enter your ruview API token (or press Enter to generate one): " API_TOKEN
    if [ -z "$API_TOKEN" ]; then
        API_TOKEN="$(cat /proc/sys/kernel/random/uuid 2>/dev/null || uuidgen)"
        info "Generated token: $API_TOKEN"
    fi

    read -rp "Enter node positions (or press Enter for 6-node default): " NODE_POSITIONS
    if [ -z "$NODE_POSITIONS" ]; then
        NODE_POSITIONS="4.5,4.5,0;2,2,3;7,2,3;4.5,7,3;2,2,6;5,5,6"
    fi

    info "Pulling ruview Docker image..."
    docker pull ruvnet/wifi-densepose:latest

    info "Starting ruview server..."
    docker run -d \
        --name ruview \
        --restart unless-stopped \
        -p 3000:3000 \
        -p 3001:3001 \
        -p 5005:5005/udp \
        -e RUVIEW_API_TOKEN="$API_TOKEN" \
        ruvnet/wifi-densepose:latest \
        ./sensing-server --http-port 3000 --ws-port 3001 --bind-addr 0.0.0.0 --source auto \
        --node-positions "$NODE_POSITIONS" \
        --no-edge-registry

    info "ruview server running."
    info "Dashboard: http://$(hostname -I | awk '{print $1}'):3000/ui/index.html"
    info "API Token: $API_TOKEN"
    echo ""
    warn "Save your API token — you will need it for the SlimeVR bridge."
    echo "$API_TOKEN" > "$SCRIPT_DIR/.api_token"
}


# ── Board flashing ────────────────────────────────────────────────────────────

flash_board() {
    section "Flash ESP32-S3 Board"

    detect_serial_port

    if [ -z "$SERIAL_PORT" ]; then
        read -rp "Enter serial port manually (e.g. /dev/ttyUSB0 or /dev/ttyS3): " SERIAL_PORT
    else
        read -rp "Use detected port $SERIAL_PORT? (Enter to confirm or type another): " OVERRIDE
        [ -n "$OVERRIDE" ] && SERIAL_PORT="$OVERRIDE"
    fi

    read -rp "Board number (1-9): " BOARD_NUM
    read -rp "WiFi SSID: " SSID
    read -rsp "WiFi Password: " WIFI_PASS
    echo ""
    read -rp "ruview server IP [$RUVIEW_SERVER]: " SERVER_IP
    [ -z "$SERVER_IP" ] && SERVER_IP="$RUVIEW_SERVER"

    info "Flashing board $BOARD_NUM on $SERIAL_PORT..."
    python3 -m esptool --chip esp32s3 --port "$SERIAL_PORT" --baud 460800 \
        write_flash \
        0x0     "$SCRIPT_DIR/bootloader.bin" \
        0x8000  "$SCRIPT_DIR/partition-table.bin" \
        0xf000  "$SCRIPT_DIR/ota_data_initial.bin" \
        0x20000 "$SCRIPT_DIR/esp32-csi-node.bin"

    info "Provisioning board $BOARD_NUM..."
    python3 "$SCRIPT_DIR/provision.py" \
        --port "$SERIAL_PORT" \
        --ssid "$SSID" \
        --password "$WIFI_PASS" \
        --target-ip "$SERVER_IP" \
        --node-id "$BOARD_NUM"

    info "Board $BOARD_NUM done. Unplug and label it."
}


# ── SlimeVR bridge ────────────────────────────────────────────────────────────

run_slimevr_bridge() {
    section "SlimeVR Body Tracking Bridge"

    API_TOKEN=""
    if [ -f "$SCRIPT_DIR/.api_token" ]; then
        API_TOKEN="$(cat "$SCRIPT_DIR/.api_token")"
        info "Using saved API token."
    else
        read -rp "Enter ruview API token: " API_TOKEN
    fi

    read -rp "SlimeVR server IP (your gaming PC): " SLIMEVR_HOST
    read -rp "ruview WebSocket URL [$RUVIEW_WS]: " WS_OVERRIDE
    [ -n "$WS_OVERRIDE" ] && RUVIEW_WS="$WS_OVERRIDE"

    info "Starting SlimeVR bridge..."
    python3 "$SCRIPT_DIR/ruview-slimevr-bridge.py" \
        --ruview-ws "$RUVIEW_WS" \
        --slimevr-host "$SLIMEVR_HOST" \
        --slimevr-port "$SLIMEVR_PORT" \
        --api-token "$API_TOKEN"
}


# ── Main menu ─────────────────────────────────────────────────────────────────

main() {
    echo ""
    echo "  ██████╗ ██╗   ██╗██╗   ██╗██╗███████╗██╗    ██╗"
    echo "  ██╔══██╗██║   ██║██║   ██║██║██╔════╝██║    ██║"
    echo "  ██████╔╝██║   ██║██║   ██║██║█████╗  ██║ █╗ ██║"
    echo "  ██╔══██╗██║   ██║╚██╗ ██╔╝██║██╔══╝  ██║███╗██║"
    echo "  ██║  ██║╚██████╔╝ ╚████╔╝ ██║███████╗╚███╔███╔╝"
    echo "  ╚═╝  ╚═╝ ╚═════╝   ╚═══╝  ╚═╝╚══════╝ ╚══╝╚══╝"
    echo ""
    echo "  WiFi Presence & Body Tracking — Setup Installer"
    echo ""

    section "Environment Detection"
    detect_env
    detect_distro
    detect_serial_group

    echo ""
    echo "What would you like to do?"
    echo "  1) Full install (dependencies + Docker + ruview server)"
    echo "  2) Flash an ESP32-S3 board"
    echo "  3) Start SlimeVR body tracking bridge"
    echo "  4) All of the above"
    echo ""
    read -rp "Choice [1-4]: " CHOICE

    case "$CHOICE" in
        1)
            install_python_deps
            install_docker
            setup_serial_permissions
            setup_ruview_server
            ;;
        2)
            install_python_deps
            setup_serial_permissions
            flash_board
            ;;
        3)
            run_slimevr_bridge
            ;;
        4)
            install_python_deps
            install_docker
            setup_serial_permissions
            setup_ruview_server
            flash_board
            run_slimevr_bridge
            ;;
        *)
            error "Invalid choice."
            ;;
    esac

    echo ""
    info "Done."
}

main "$@"
