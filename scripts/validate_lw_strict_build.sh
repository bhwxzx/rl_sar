#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd "${script_dir}/.." && pwd)"
strict_jobs="${LW_STRICT_BUILD_JOBS:-2}"

case "${strict_jobs}" in
    ''|*[!0-9]*)
        echo "LW_STRICT_BUILD_JOBS must be a positive integer" >&2
        exit 2
        ;;
esac
if [ "${strict_jobs}" -le 0 ]; then
    echo "LW_STRICT_BUILD_JOBS must be a positive integer" >&2
    exit 2
fi

strict_build_dir="$(mktemp -d /tmp/lw-strict-build.XXXXXX)"
cleanup() {
    cmake -E remove_directory "${strict_build_dir}"
}
trap cleanup EXIT

cmake \
    -S "${repository_root}/src/rl_sar" \
    -B "${strict_build_dir}" \
    -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DLW_STRICT_WARNINGS=ON
cmake --build "${strict_build_dir}" --parallel "${strict_jobs}"
ctest --test-dir "${strict_build_dir}" --output-on-failure
