#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Usage: $0 <onnx|libtorch> <runtime-directory> [architecture]" >&2
    exit 2
fi

runtime_kind=$1
runtime_dir=$2
expected_arch=${3:-$(uname -m)}
validation_os=${RL_SAR_RUNTIME_VALIDATION_OS:-$(uname -s)}

case "$runtime_kind" in
    onnx)
        required_header="$runtime_dir/include/onnxruntime_cxx_api.h"
        shopt -s nullglob
        library_candidates=(
            "$runtime_dir/lib/libonnxruntime.so"
            "$runtime_dir/lib"/libonnxruntime.so.*
        )
        shopt -u nullglob
        ;;
    libtorch)
        if [[ -f "$runtime_dir/include/torch/torch.h" ]]; then
            required_header="$runtime_dir/include/torch/torch.h"
        else
            required_header="$runtime_dir/include/torch/csrc/api/include/torch/torch.h"
        fi
        library_candidates=("$runtime_dir/lib/libtorch_cpu.so")
        ;;
    *)
        echo "Unsupported inference runtime kind: $runtime_kind" >&2
        exit 2
        ;;
esac

if [[ ! -f "$required_header" ]]; then
    echo "$runtime_kind header is missing: $required_header" >&2
    exit 1
fi

# Keep the previous include/lib structural contract on non-Linux development
# hosts. ELF machine validation below is deliberately Linux-specific.
if [[ "$validation_os" != Linux ]]; then
    if [[ ! -d "$runtime_dir/lib" ]]; then
        echo "$runtime_kind library directory is missing: $runtime_dir/lib" >&2
        exit 1
    fi
    echo "$runtime_kind runtime structure is valid for $validation_os"
    exit 0
fi

runtime_library=""
for candidate in "${library_candidates[@]}"; do
    if [[ -f "$candidate" ]]; then
        runtime_library=$candidate
        break
    fi
done
if [[ -z "$runtime_library" ]]; then
    echo "$runtime_kind shared library is missing under: $runtime_dir/lib" >&2
    exit 1
fi

if ! command -v readelf >/dev/null 2>&1; then
    echo "readelf is required to validate Linux inference libraries" >&2
    exit 1
fi

case "$expected_arch" in
    x86_64|amd64)
        expected_machine="Advanced Micro Devices X86-64"
        normalized_arch=x86_64
        ;;
    aarch64|arm64)
        expected_machine=AArch64
        normalized_arch=aarch64
        ;;
    *)
        echo "Unsupported Linux inference architecture: $expected_arch" >&2
        exit 2
        ;;
esac

machine=$(LC_ALL=C readelf -h "$runtime_library" 2>/dev/null \
    | awk -F: '/^[[:space:]]*Machine:/{sub(/^[[:space:]]+/, "", $2); print $2; exit}' \
    || true)
if [[ -z "$machine" ]]; then
    echo "Cannot read ELF machine type from: $runtime_library" >&2
    exit 1
fi
if [[ "$machine" != "$expected_machine" ]]; then
    echo "$runtime_kind architecture mismatch: expected $normalized_arch" \
         "($expected_machine), found $machine in $runtime_library" >&2
    exit 1
fi

echo "$runtime_kind runtime is valid for $normalized_arch: $runtime_library"
