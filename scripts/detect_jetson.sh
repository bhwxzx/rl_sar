#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

resolve_jetson_platform
printf '%s;%s\n' "$IS_JETSON" "$JETSON_DETECTION_SOURCE"
