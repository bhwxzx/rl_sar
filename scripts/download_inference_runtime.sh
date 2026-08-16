#!/usr/bin/env bash

# Install only the reviewed inference-runtime archives pinned in
# scripts/inference_runtime_archives.json.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

source "${SCRIPT_DIR}/common.sh"

resolve_jetson_platform
print_info "Jetson mode: ${IS_JETSON} (${JETSON_DETECTION_SOURCE})"

OS_TYPE="${RL_SAR_PLATFORM_OS:-$(uname -s)}"
ARCH_TYPE="${RL_SAR_PLATFORM_ARCH:-$(uname -m)}"

if [[ $# -ge 1 && ( "$1" == "libtorch" || "$1" == "all" ) ]]; then
    print_error "Download target must be onnx"
    exit 2
fi

if [[ $# -eq 0 ]]; then
    TARGET_DIR="library/inference_runtime"
    DOWNLOAD_TARGET="onnx"
elif [[ $# -eq 1 ]]; then
    if [[ "$1" == "onnx" ]]; then
        TARGET_DIR="library/inference_runtime"
        DOWNLOAD_TARGET="$1"
    else
        TARGET_DIR="$1"
        DOWNLOAD_TARGET="onnx"
    fi
elif [[ $# -eq 2 ]]; then
    TARGET_DIR="$1"
    DOWNLOAD_TARGET="$2"
else
    print_error "Usage: $0 [target-directory] [onnx]"
    exit 2
fi

if [[ "$DOWNLOAD_TARGET" != onnx ]]; then
    print_error "Download target must be onnx"
    exit 2
fi

if [[ "$TARGET_DIR" = /* ]]; then
    MODEL_INTERFACE_DIR="$TARGET_DIR"
else
    MODEL_INTERFACE_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
fi
mkdir -p "$MODEL_INTERFACE_DIR"

ONNXRUNTIME_VERSION="1.22.0"
RUNTIME_VALIDATOR="${SCRIPT_DIR}/validate_inference_runtime.sh"
RUNTIME_MANAGER="${SCRIPT_DIR}/manage_inference_runtime.py"
RUNTIME_CATALOG="${SCRIPT_DIR}/inference_runtime_archives.json"

manager_arguments() {
    local kind=$1
    local version=$2
    printf '%s\0' \
        --catalog "$RUNTIME_CATALOG" \
        --kind "$kind" \
        --version "$version" \
        --os "$OS_TYPE" \
        --architecture "$ARCH_TYPE"
}

download_archive() {
    local destination=$1
    local url=$2
    if command -v curl &>/dev/null; then
        curl -fL --progress-bar -o "$destination" "$url"
    elif command -v wget &>/dev/null; then
        wget --show-progress -O "$destination" "$url"
    else
        print_error "curl or wget is required to download inference runtimes"
        return 1
    fi
}

ensure_runtime() {
    local kind=$1
    local version=$2
    local common=()
    while IFS= read -r -d '' argument; do
        common+=("$argument")
    done < <(manager_arguments "$kind" "$version")

    local check_status=0
    if python3 "$RUNTIME_MANAGER" check \
            "${common[@]}" \
            --runtime-root "$MODEL_INTERFACE_DIR" \
            --validator "$RUNTIME_VALIDATOR"; then
        print_success "$kind $version already exists with approved provenance"
        return
    else
        check_status=$?
    fi

    if [[ $check_status -eq 3 ]]; then
        print_error "Existing $kind runtime is valid or identified but is not the approved $version archive"
        print_info "The directory was preserved. Review any runtime upgrade as a separate change."
        return 1
    fi
    if [[ $check_status -ne 4 ]]; then
        print_error "Could not validate the pinned $kind runtime selection"
        return 1
    fi

    local selection
    if ! selection=$(python3 "$RUNTIME_MANAGER" select "${common[@]}"); then
        return 1
    fi
    local archive_name url expected_digest extra
    IFS=$'\t' read -r archive_name url expected_digest extra <<< "$selection"
    if [[ -z "$archive_name" || -z "$url" || -z "$expected_digest" || -n "${extra:-}" ]]; then
        print_error "Pinned runtime selection returned malformed data"
        return 1
    fi

    local archive_path
    archive_path=$(mktemp "${MODEL_INTERFACE_DIR}/.${archive_name}.XXXXXX")
    print_info "Downloading approved $kind $version for ${OS_TYPE} ${ARCH_TYPE}"
    print_info "URL: $url"
    print_info "Expected SHA-256: $expected_digest"
    if ! download_archive "$archive_path" "$url"; then
        rm -f -- "$archive_path"
        print_error "Download failed; the existing runtime was preserved"
        return 1
    fi
    if ! python3 "$RUNTIME_MANAGER" install \
            "${common[@]}" \
            --runtime-root "$MODEL_INTERFACE_DIR" \
            --validator "$RUNTIME_VALIDATOR" \
            --archive "$archive_path"; then
        rm -f -- "$archive_path"
        print_error "Archive validation or candidate installation failed; the existing runtime was preserved"
        return 1
    fi
    rm -f -- "$archive_path"

    python3 "$RUNTIME_MANAGER" check \
        "${common[@]}" \
        --runtime-root "$MODEL_INTERFACE_DIR" \
        --validator "$RUNTIME_VALIDATOR"
    print_success "$kind $version installed with approved provenance"
}

ensure_runtime onnx "$ONNXRUNTIME_VERSION"

print_success "All requested inference runtimes are ready"
