#!/usr/bin/env bash
set -euo pipefail

bundle_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${bundle_root}"
if [[ -x .venv/bin/python ]]; then
    exec .venv/bin/python app/capture_live_camera.py "$@"
fi
exec python3 app/capture_live_camera.py "$@"
