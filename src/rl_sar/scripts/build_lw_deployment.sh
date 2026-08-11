#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 <empty-output-prefix> [commit]" >&2
    exit 2
fi

repository_root=$(git rev-parse --show-toplevel)
output_prefix=$(realpath -m "$1")
source_revision=${2:-HEAD}
source_commit=$(git -C "$repository_root" rev-parse --verify "${source_revision}^{commit}")
inference_runtime="$repository_root/library/inference_runtime"

if [[ "$output_prefix" == "/" || "$output_prefix" == "$repository_root" ]]; then
    echo "Refusing unsafe output prefix: $output_prefix" >&2
    exit 2
fi
if [[ -e "$output_prefix" ]] && find "$output_prefix" -mindepth 1 -print -quit | grep -q .; then
    echo "Output prefix must not exist or must be empty: $output_prefix" >&2
    exit 2
fi
if [[ ! -d "$inference_runtime/onnxruntime" ]]; then
    echo "ONNX Runtime dependency is missing: $inference_runtime/onnxruntime" >&2
    exit 1
fi

temporary_root=$(mktemp -d)
source_tree="$temporary_root/source"

cleanup()
{
    if [[ -e "$source_tree/.git" ]]; then
        git -C "$repository_root" worktree remove --force "$source_tree" >/dev/null
    fi
    rm -rf -- "$temporary_root"
}
trap cleanup EXIT

git -C "$repository_root" worktree add --detach "$source_tree" "$source_commit"
git -C "$source_tree" submodule update --init --recursive
if [[ -n $(git -C "$source_tree" status --porcelain --untracked-files=all) ]]; then
    echo "Temporary source worktree is not clean" >&2
    exit 1
fi
bash "$source_tree/scripts/validate_inference_runtime.sh" \
    onnx "$inference_runtime/onnxruntime" "$(uname -m)"
bash "$source_tree/scripts/validate_lw_description.sh"

mkdir -p "$output_prefix"
colcon --log-base "$temporary_root/log" build \
    --base-paths "$source_tree/src" \
    --packages-up-to rl_sar fdilink_ahrs \
    --merge-install \
    --build-base "$temporary_root/build" \
    --install-base "$output_prefix" \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DLW_PRODUCTION_DEPLOYMENT=ON \
        -DINFERENCE_RUNTIME_DIR="$inference_runtime" \
        -DBUILD_TESTING=OFF

manifest="$output_prefix/share/rl_sar/deployment/LW/manifest.yaml"
if [[ ! -f "$manifest" || -L "$manifest" ]]; then
    echo "Deployment manifest was not produced: $manifest" >&2
    exit 1
fi
python3 "$source_tree/src/rl_sar/scripts/verify_lw_policy_parity.py" \
    --policy-root "$source_tree/policy" \
    --manifest "$manifest"
if find "$output_prefix/lib/rl_sar/rl_real_LW" \
        "$output_prefix/share/rl_sar/deployment/LW" -type l -print -quit \
        | grep -q .; then
    echo "Production bundle contains symbolic links" >&2
    exit 1
fi
LW_DEPLOYMENT_PREFIX="$output_prefix" bash -c '
    source "$LW_DEPLOYMENT_PREFIX/setup.bash"
    for package in serial fdilink_ahrs rl_sar; do
        package_prefix=$(ros2 pkg prefix "$package")
        if [[ $(realpath -m "$package_prefix") != "$LW_DEPLOYMENT_PREFIX" ]]; then
            echo "Package $package did not resolve inside the deployment prefix: $package_prefix" >&2
            exit 1
        fi
    done
    for executable in \
        "$LW_DEPLOYMENT_PREFIX/lib/fdilink_ahrs/ahrs_driver_node" \
        "$LW_DEPLOYMENT_PREFIX/lib/rl_sar/rl_real_LW"; do
        ldd_output=$(ldd "$executable")
        if grep -q "not found" <<< "$ldd_output"; then
            echo "Deployment executable has unresolved libraries: $executable" >&2
            printf "%s\n" "$ldd_output" >&2
            exit 1
        fi
        if [[ "$executable" == */lib/rl_sar/rl_real_LW ]] \
           && grep -Eq "libtorch|libc10" <<< "$ldd_output"; then
            echo "LW production executable unexpectedly depends on LibTorch" >&2
            printf "%s\n" "$ldd_output" >&2
            exit 1
        fi
    done
    "$LW_DEPLOYMENT_PREFIX/lib/rl_sar/rl_real_LW" \
        --verify-deployment-only
'

echo "LW deployment created from commit $source_commit"
echo "Install prefix: $output_prefix"
echo "Manifest: $manifest"
echo "Launch from this prefix with:"
echo "  source '$output_prefix/setup.bash' && ros2 launch rl_sar rl_real_LW.launch.py"
