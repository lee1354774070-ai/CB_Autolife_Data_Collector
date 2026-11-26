"""Read the S3 HangKai and Agile hand sensors for camera-synchronized recording."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Event, Lock, Thread
import time

import numpy as np

from time_sync import EPOCH_SEC_MAX, EPOCH_SEC_MIN, nearest_sample

FORCE_AXES = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
TACTILE_AXES = ("Fx_N", "Fy_N", "Fz_N", "normal_N", "tangential_N", "direction_deg")
TACTILE_IDS = {"left": (0, 1), "right": (2, 3)}


def tactile_points(side):
    return [(sensor_id, point) for sensor_id in TACTILE_IDS[side] for point in range(4)]


def feature(names, dtype="float32"):
    return {"dtype": dtype, "shape": (len(names),), "names": names}


@dataclass(frozen=True)
class SensorSample:
    stamp_sec: float
    stamps: np.ndarray
    values: np.ndarray
    status: np.ndarray | None = None

    def freshness_error(self, now, max_age):
        newest = float(self.stamps.max())
        age = now - float(self.stamps.min())
        if newest > now:
            return f"host timestamp is {(newest - now) * 1000:.1f} ms in the future"
        if age > max_age:
            return f"no fresh sample: oldest point is {age:.3f} s old (limit {max_age:.3f} s)"
        return None


def read_field(frame, name, shape):
    key = (0, name)
    values = frame.numpy(*key)
    if values.shape != shape or frame.fields[key].validity != b"\x01" * shape[0]:
        raise ValueError(f"invalid {name}, expected shape {shape}")
    return values


def decode_packet(frame, kind, side):
    if kind == "force":
        linear = read_field(frame, "force", (1, 3))
        moment = read_field(frame, "torque", (1, 3))
        values = np.concatenate((linear[0], moment[0])).astype(np.float32)
        stamps = read_field(frame, "host_receipt_epoch_s", (1,)).copy()
        status = None
    else:
        layout = tactile_points(side)
        rows = (len(layout),)
        sensor_ids = read_field(frame, "logical_sensor_id", rows)
        points = read_field(frame, "point_index", rows)
        pairs = list(zip(sensor_ids.tolist(), points.tolist()))
        order = sorted(range(len(pairs)), key=pairs.__getitem__)
        if [pairs[i] for i in order] != layout:
            raise ValueError(f"expected Agile {side} sensor/point layout {layout}")
        xyz = read_field(frame, "contact_force_xyz", (*rows, 3))
        normal = read_field(frame, "normal_force", rows)
        tangent = read_field(frame, "tangential_force", rows)
        direction = read_field(frame, "tangential_direction", rows)
        values = np.column_stack((xyz, normal, tangent, direction))[order].astype(np.float32).reshape(-1)
        stamps = read_field(frame, "host_receipt_epoch_s", rows)[order]
        status = read_field(frame, "status", rows)[order].astype(np.int64)
    if not np.isfinite(values).all() or not ((EPOCH_SEC_MIN <= stamps) & (stamps <= EPOCH_SEC_MAX)).all():
        raise ValueError("non-finite measurements or missing host epoch timestamps")
    return SensorSample((float(stamps.min()) + float(stamps.max())) / 2, stamps, values, status)


class HandSensorCapture:
    def __init__(self, with_force, with_tactile, buffer_size):
        try:
            from autolife_robot_sdk.shm import ReadStatus, ShmError
            if with_force:
                from autolife_robot_sdk.shm.force_rs import ForceReader
            if with_tactile:
                from autolife_robot_sdk.shm.tactile_rs import TactileReader
        except ImportError as exc:
            raise RuntimeError("Hand sensor recording requires the Autolife SDK 3.x in LEROBOT_PY") from exc

        self.shm_error = ShmError
        self.lost = (ReadStatus.CLOSED, ReadStatus.OWNER_LOST)
        self.lock = Lock()
        self.stopping = Event()
        self.worker = None
        self.streams = {}
        self.samples = {}
        self.errors = {}
        self.features = {}
        try:
            for kind, model, reader_type in (
                ("force", "hangkai", ForceReader if with_force else None),
                ("tactile", "agile", TactileReader if with_tactile else None),
            ):
                if reader_type is None:
                    continue
                for side in TACTILE_IDS:
                    key = f"observation.{kind}.{side}"
                    prefix = "force_sensor" if kind == "force" else "tactile"
                    name = f"alshm_{prefix}_hand_{side}_{model}"
                    try:
                        self.streams[key] = (kind, side, reader_type.open(name, start="latest"))
                    except (OSError, ShmError) as exc:
                        raise RuntimeError(f"Required hand sensor {name} is unavailable: {exc}") from exc
                    self.samples[key] = deque(maxlen=buffer_size)
                    if kind == "force":
                        self.features[key] = feature(FORCE_AXES)
                        self.features[key + "_sync_delta_ms"] = feature(["sensor_minus_image_ms"])
                    else:
                        labels = [f"sensor{sid}.point{pid}" for sid, pid in tactile_points(side)]
                        self.features[key] = feature([f"{label}.{axis}" for label in labels for axis in TACTILE_AXES])
                        self.features[key + "_status"] = feature(labels, "int64")
                        self.features[key + "_sync_delta_ms"] = feature(labels)
        except Exception:
            self.close()
            raise

    def start(self, fps):
        def run():
            interval = 1.0 / fps
            while not self.stopping.is_set():
                tick = time.monotonic()
                self.poll()
                self.stopping.wait(max(0.001, interval - (time.monotonic() - tick)))

        worker = Thread(target=run, name="hand-sensors", daemon=True)
        worker.start()
        self.worker = worker

    def poll(self):
        for key, (kind, side, reader) in self.streams.items():
            try:
                result = reader.read(mode="latest", timeout=0)
                if result.status in self.lost:
                    raise ValueError(f"publisher {result.status.value}")
                if result.frame is None:
                    continue
                sample = decode_packet(result.frame, kind, side)
                with self.lock:
                    ring = self.samples[key]
                    # SDK latest can return the same publication on every poll,
                    # even after its producer stalls. Never evict useful history
                    # or manufacture a new measurement from a repeated timestamp.
                    if not ring or not np.array_equal(ring[-1].stamps, sample.stamps):
                        ring.append(sample)
                    self.errors.pop(key, None)
            except (OSError, ValueError, KeyError, StopIteration, self.shm_error) as exc:
                with self.lock:
                    self.samples[key].clear()
                    self.errors[key] = str(exc)

    def problems(self, now, max_age):
        with self.lock:
            faults = []
            for key, samples in self.samples.items():
                reason = (samples[-1].freshness_error(now, max_age) if samples
                          else self.errors.get(key, "no fresh sample received"))
                if reason:
                    faults.append(f"{key}: {reason}")
            return faults

    def match(self, anchor_sec, now, max_delta, max_age):
        values, deltas_ms = {}, {}
        with self.lock:
            snapshot = [(key, tuple(samples), self.errors.get(key, "no fresh sample"))
                        for key, samples in self.samples.items()]
        for key, samples, fault in snapshot:
            matched = nearest_sample(samples, anchor_sec)
            reason = matched[0].freshness_error(now, max_age) if matched is not None else fault
            if reason:
                raise ValueError(f"{key}: {reason}")
            sample, _ = matched
            offsets = sample.stamps - anchor_sec
            largest = offsets[np.argmax(np.abs(offsets))]
            if abs(largest) > max_delta:
                raise ValueError(f"{key}: timestamp offset {largest * 1000:.1f} ms exceeds sync limit")
            values[key] = sample.values
            values[key + "_sync_delta_ms"] = (offsets * 1000).astype(np.float32)
            if sample.status is not None:
                values[key + "_status"] = sample.status
            deltas_ms[key] = float(largest) * 1000
        return values, deltas_ms

    def close(self):
        self.stopping.set()
        if self.worker is not None:
            self.worker.join(timeout=2.0)
            if self.worker.is_alive():
                return  # Never close an active native read during shutdown.
        for _, _, reader in self.streams.values():
            reader.close()
        self.streams.clear()
