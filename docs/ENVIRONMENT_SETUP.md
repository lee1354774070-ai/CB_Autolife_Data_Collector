# One-command Collector environment setup

Run from the Collector directory **on the robot**. Requires Linux x86_64,
Conda, and an existing ROS 2 Jazzy installation. Python 3.12 is used. This is
not a Jetson/Thor ARM installer.

```bash
python3 scripts/setup_collector_env.py
```

Creates an isolated environment named `collector`. `auto` chooses NVIDIA
CUDA packages when `nvidia-smi` detects an available device, otherwise CPU
packages. A failed GPU installation/probe fails visibly; it is not silently
reported as GPU success.

```bash
# New GPU environment, fail when NVIDIA hardware/driver is unavailable
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg

# CPU-only environment
python3 scripts/setup_collector_env.py --name collector_cpu --backend cpu

# Clone existing dependencies without modifying the source environment
python3 scripts/setup_collector_env.py --name collector_gpu --clone lerobot --backend nvjpeg

# Preview without installing or writing files
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg --dry-run

# Verify an existing environment without changing its packages/configuration
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg --check

python3 scripts/setup_collector_env.py --help
```

Use `--conda /path/bin/conda` or `--ros-setup /path/setup.bash` for nonstandard
installations. The target is `<conda-base>/envs/<name>`; `--clone` accepts an
environment name or absolute prefix.

## Safety and recovery

Existing environments are never overwritten. `base`, `lerobot`, `robot_env`,
and `cuda_env` are protected installation targets, but may be checked.
Concurrent installers for the same target are rejected. A failed installation
is retained, not deleted. Append `--resume` to the **same original command**
only for an incomplete environment owned by this installer. If environment
creation failed before Python was created, choose another name and inspect
the incomplete directory manually.

The script does not install Conda, ROS, SDKs, NVIDIA drivers, factory teleop or
DAgger dependencies. It does not change system packages, restart services,
read hardware cameras, publish motion commands or edit shell startup files.

## Packages and checks

Pinned capture baseline: LeRobot 0.6.0, torch 2.7.1, torchvision 0.22.1,
datasets 4.8.5, PyArrow 25.0.0, PyAV 15.1.0, NumPy 2.2.6 and OpenCV headless
4.13.0.92. CUDA uses `cu126`, CPU uses `cpu`, following the
[official PyTorch pairing](https://pytorch.org/get-started/previous-versions/#v2-7-1).
Transitive dependencies are still resolved by pip; this is not a complete
offline lockfile. Training extras and torchcodec are not needed or installed.
The preflight requires at least 16 GiB free for GPU or 6 GiB for CPU. This is
a minimum check, not a quota; large clone sources may need more space.

Checks include `pip check`, real recorder/ROS imports, an actual GPU JPEG
probe when selected, and saving/decoding a tiny synthetic LeRobot dataset in
a temporary directory. No ROS node is created. These checks do not certify
real camera synchronization, VR controls, DAgger, depth video or hardware
video encoding. JPEG decoding and NVENC/VAAPI video encoding remain separate.

## Use the environment

A successful installation sets `LEROBOT_PY` in that environment's Conda
configuration only. Reactivate it, retaining your usual mode/task options:

```bash
conda activate collector_gpu
JPEG_DECODER=nvjpeg TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water
```

Or specify the interpreter without activation:

```bash
LEROBOT_PY=/home/ubuntu/miniconda3/envs/collector_gpu/bin/python \
JPEG_DECODER=nvjpeg TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water
```

Adjust the path for your Conda installation. `nvjpeg` requires GPU success;
the recorder's default `auto` permits CPU fallback at startup. Keep
`ROBOT_PY` pointing to the existing factory `robot_env` for VR/camera helpers.
`--check` does not configure `LEROBOT_PY` for externally created environments;
use the explicit path in that case.

## Validation scope

Current merge tests and limitations are recorded in [VALIDATION](VALIDATION.md).
Environment probes use synthetic inputs and do not certify real teleoperation or DAgger motion.
