#!/usr/bin/env bash
# Check or install the zsibot SDK (genisom_L1_sdk) that l1w_driver links against.
#
#   script/setup.sh check     # exit 0 when headers and libzsibot.a are installed
#   script/setup.sh install   # clone, build and install into ZSIBOT_SDK_ROOT
#
# Env: ZSIBOT_SDK_REPO, ZSIBOT_SDK_REF (main), ZSIBOT_SDK_ROOT (/usr/local), ROBOT_IP
set -euo pipefail

SDK_REPO="${ZSIBOT_SDK_REPO:-https://github.com/zsibot/genisom_l1_sdk.git}"
SDK_REF="${ZSIBOT_SDK_REF:-main}"
PREFIX="${ZSIBOT_SDK_ROOT:-/usr/local}"
ROBOT_IP="${ROBOT_IP:-192.168.234.1}"
SUDO=$([[ "$(id -u)" -eq 0 || -w "$PREFIX" ]] && echo "" || echo "sudo")

check() {
    if [[ -f "$PREFIX/include/zsibot_sdk/zsibot_api.h" && -f "$PREFIX/lib/libzsibot.a" ]]; then
        local robot
        robot=$(ping -c1 -W1 "$ROBOT_IP" >/dev/null 2>&1 && echo "up" || echo "down")
        echo "zsibot SDK in $PREFIX, robot $ROBOT_IP $robot"
        return 0
    fi
    echo "zsibot SDK not installed in $PREFIX"
    return 1
}

install() {
    local tmp
    tmp=$(mktemp -d)
    git clone --depth 1 --branch "$SDK_REF" "$SDK_REPO" "$tmp"
    cmake -S "$tmp" -B "$tmp/build" -DCMAKE_INSTALL_PREFIX="$PREFIX"
    cmake --build "$tmp/build" -j"$(nproc)"
    $SUDO cmake --install "$tmp/build"
    rm -rf "$tmp"
    check
}

case "${1:-check}" in
    check) check ;;
    install) install ;;
    *) sed -n '2,8p' "$0"; exit 1 ;;
esac
