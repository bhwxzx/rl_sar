#!/usr/bin/env bash

# Install only the reviewed MuJoCo 3.2.7 archives pinned in
# scripts/mujoco_archives.json.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

source "${SCRIPT_DIR}/common.sh"

if [[ $# -gt 1 ]]; then
    print_error "Usage: $0 [target-directory]"
    exit 2
fi

TARGET_DIR="${1:-library}"
if [[ "$TARGET_DIR" = /* ]]; then
    MUJOCO_BASE_DIR="$TARGET_DIR"
else
    MUJOCO_BASE_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
fi
mkdir -p "$MUJOCO_BASE_DIR"
if [[ -L "$MUJOCO_BASE_DIR" || ! -d "$MUJOCO_BASE_DIR" ]]; then
    print_error "MuJoCo target root must be a regular directory: $MUJOCO_BASE_DIR"
    exit 1
fi

RAW_OS="${RL_SAR_PLATFORM_OS:-$(uname -s)}"
ARCH_TYPE="${RL_SAR_PLATFORM_ARCH:-$(uname -m)}"
case "$RAW_OS" in
    Linux|Darwin)
        OS_TYPE="$RAW_OS"
        ;;
    MINGW*|MSYS*|CYGWIN*|Windows)
        OS_TYPE="Windows"
        ;;
    *)
        print_error "Unsupported operating system: $RAW_OS"
        exit 1
        ;;
esac

MUJOCO_VERSION="3.2.7"
MUJOCO_MANAGER="${SCRIPT_DIR}/manage_mujoco.py"
MUJOCO_CATALOG="${SCRIPT_DIR}/mujoco_archives.json"
MANAGER_ARGUMENTS=(
    --catalog "$MUJOCO_CATALOG"
    --version "$MUJOCO_VERSION"
    --os "$OS_TYPE"
    --architecture "$ARCH_TYPE"
)

download_archive() {
    local destination=$1
    local url=$2
    if command -v curl &>/dev/null; then
        curl -fL --progress-bar -o "$destination" "$url"
    elif command -v wget &>/dev/null; then
        wget --show-progress -O "$destination" "$url"
    else
        print_error "curl or wget is required to download MuJoCo"
        return 1
    fi
}

print_header "[MuJoCo Setup]"
print_info "Platform: ${OS_TYPE} ${ARCH_TYPE}"

check_status=0
if python3 "$MUJOCO_MANAGER" check \
        "${MANAGER_ARGUMENTS[@]}" \
        --runtime-root "$MUJOCO_BASE_DIR"; then
    print_success "MuJoCo ${MUJOCO_VERSION} is already installed from the approved archive"
    exit 0
else
    check_status=$?
fi
if [[ $check_status -eq 3 ]]; then
    print_error "The existing MuJoCo destination is unsafe and was preserved"
    exit 1
fi
if [[ $check_status -ne 4 ]]; then
    print_error "Could not validate the pinned MuJoCo selection"
    exit 1
fi

selection="$(python3 "$MUJOCO_MANAGER" select "${MANAGER_ARGUMENTS[@]}")"
archive_name=""
url=""
expected_digest=""
extra=""
IFS=$'\t' read -r archive_name url expected_digest extra <<< "$selection"
if [[ -z "$archive_name" || -z "$url" || -z "$expected_digest" || -n "$extra" ]]; then
    print_error "Pinned MuJoCo selection returned malformed data"
    exit 1
fi

archive_path="$(mktemp "${MUJOCO_BASE_DIR}/.${archive_name}.XXXXXX")"
cleanup_archive() {
    if [[ -n "${archive_path:-}" && -f "$archive_path" && ! -L "$archive_path" ]]; then
        rm -f -- "$archive_path"
    fi
}
trap cleanup_archive EXIT

print_info "Downloading approved MuJoCo ${MUJOCO_VERSION}"
print_info "URL: $url"
print_info "Expected SHA-256: $expected_digest"
if ! download_archive "$archive_path" "$url"; then
    print_error "Download failed; the existing MuJoCo installation was preserved"
    exit 1
fi

if ! python3 "$MUJOCO_MANAGER" install \
        "${MANAGER_ARGUMENTS[@]}" \
        --runtime-root "$MUJOCO_BASE_DIR" \
        --archive "$archive_path"; then
    print_error "Archive validation or installation failed; the existing MuJoCo installation was preserved"
    exit 1
fi

python3 "$MUJOCO_MANAGER" check \
    "${MANAGER_ARGUMENTS[@]}" \
    --runtime-root "$MUJOCO_BASE_DIR"

cleanup_archive
trap - EXIT
print_separator
print_success "MuJoCo ${MUJOCO_VERSION} setup completed successfully"
print_info "Installation path: ${MUJOCO_BASE_DIR}/mujoco"
