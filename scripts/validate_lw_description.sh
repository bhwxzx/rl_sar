#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/common.sh"

TARGET_DIR="${1:-src/rl_sar_zoo}"
if [[ "$TARGET_DIR" = /* ]]; then
    ROBOT_DESC_DIR="$TARGET_DIR"
else
    ROBOT_DESC_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
fi

MANIFEST="${ROBOT_DESC_DIR}/LW_DESCRIPTION_MANIFEST.sha256"

print_header "[Validating LW Description]"

if [ ! -d "${ROBOT_DESC_DIR}/LW_description" ]; then
    print_error "Missing vendored LW description: ${ROBOT_DESC_DIR}/LW_description"
    exit 1
fi

if find "$ROBOT_DESC_DIR" -name .git -print -quit | grep -q .; then
    print_error "Nested Git metadata is not allowed in ${ROBOT_DESC_DIR}"
    exit 1
fi

unexpected_description="$(
    find "$ROBOT_DESC_DIR" -mindepth 1 -maxdepth 1 -type d \
        -name '*_description' ! -name LW_description -print -quit
)"
if [ -n "$unexpected_description" ]; then
    print_error "Non-LW robot description found: ${unexpected_description}"
    exit 1
fi

for entry in "$ROBOT_DESC_DIR"/*; do
    case "$(basename "$entry")" in
        LW_description|LW_DESCRIPTION_MANIFEST.sha256|README.md) ;;
        *)
            print_error "Unexpected rl_sar_zoo entry: ${entry}"
            exit 1
            ;;
    esac
done

if [ ! -f "$MANIFEST" ]; then
    print_error "Missing LW description manifest: ${MANIFEST}"
    exit 1
fi

if ! (cd "$ROBOT_DESC_DIR" && sha256sum -c "$(basename "$MANIFEST")"); then
    print_error "LW description manifest verification failed"
    exit 1
fi

print_success "Vendored LW description is complete and verified"
print_info "Description path: ${ROBOT_DESC_DIR}/LW_description"
