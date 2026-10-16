#!/usr/bin/env bash
# The launcher selects the CPU mask for this robot; 12-19 is the legacy S2 default.
# Apply before Python imports: all library threads/children inherit this mask.
set -euo pipefail
cpus="${HG_DAGGER_RECORDER_CPUS:-12-19}"
if [ "$#" -eq 0 ]; then
  echo "Usage: $0 PYTHON SCRIPT [ARGS...]" >&2
  exit 2
fi
# Refuse invalid masks instead of silently running unrestricted.
taskset --cpu-list "${cpus}" true
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1 BLIS_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE
echo "Recorder isolation: CPUs=${cpus}, nice=5, library threads=1"
exec nice -n 5 taskset --cpu-list "${cpus}" "$@"
