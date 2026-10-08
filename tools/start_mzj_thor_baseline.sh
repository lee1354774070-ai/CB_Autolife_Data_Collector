#!/usr/bin/env bash
# Compatibility entry; use start_thor_baseline.sh for new deployments.
set -euo pipefail
exec bash "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/start_thor_baseline.sh" "$@"
