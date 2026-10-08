"""Read the robot head RGB-D colour stream without owning camera hardware."""

from collections import deque
import threading
import time


class RgbdColorCamera:
    """Latest-frame JPEG adapter for the SDK camera shared-memory stream.

    The vendor vision service remains the only camera producer.  This class is
    a read-only consumer and keeps only the newest encoded frame, so a slow VR
    client can never create a delayed image queue.
    """

    def __init__(
            self, *, robot_model='autolife_s1', robot_version='robot_v2_2',
            module_name='mod_camera_rgbd_head', output_name='color',
            maximum_fps=20.0, jpeg_quality=75):
        self.robot_model = str(robot_model)
        self.robot_version = str(robot_version)
        self.module_name = str(module_name)
        self.output_name = str(output_name)
        self.maximum_fps = max(1.0, min(30.0, float(maximum_fps)))
        self.jpeg_quality = max(40, min(95, int(jpeg_quality)))
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None
        self._consumer = None
        self._bgr = None
        self._jpeg = None
        self._frame_id = -1
        self._jpeg_frame_id = -1
        self._jpeg_demand_until = 0.0
        self._meta = {}
        self._last_frame_time = 0.0
        self._frame_times = deque(maxlen=30)
        self._last_error = ''

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name='rgbd-head-color-jpeg', daemon=True
        )
        self._thread.start()

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_consumer()

    def snapshot(self, after_frame_id=-1, timeout=1.0):
        """Return the newest ``(jpeg, frame_id, status)`` after a frame id."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            # JPEG is retained only for the optional desktop monitor/API.
            # The VR path consumes raw latest frames over WebRTC, so do not
            # spend CPU encoding JPEG continuously when nobody requests it.
            self._jpeg_demand_until = max(
                self._jpeg_demand_until, deadline + 0.25
            )
            while (
                    not self._stop.is_set()
                    and self._jpeg_frame_id <= int(after_frame_id)
                    and time.monotonic() < deadline):
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic())
                )
            jpeg = self._jpeg
            frame_id = self._jpeg_frame_id
        return jpeg, frame_id, self.status()

    def raw_snapshot(self, after_frame_id=-1, timeout=1.0):
        """Return the newest immutable BGR frame for a realtime video track.

        Only one owned copy is made in the camera reader thread.  WebRTC then
        consumes that latest reference directly, avoiding JPEG encode/decode,
        HTTP multipart framing and a browser canvas copy.
        """
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            while (
                    not self._stop.is_set()
                    and self._frame_id <= int(after_frame_id)
                    and time.monotonic() < deadline):
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic())
                )
            bgr = self._bgr
            frame_id = self._frame_id
        return bgr, frame_id, self.status()

    def status(self):
        with self._condition:
            times = tuple(self._frame_times)
            age = (
                None if self._last_frame_time <= 0.0
                else max(0.0, time.monotonic() - self._last_frame_time)
            )
            meta = dict(self._meta)
            frame_id = self._frame_id
            error = self._last_error
        fps = 0.0
        if len(times) >= 2 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
        return {
            'name': '头部深度相机 RGB',
            'image_key': 'rgbd_head_color',
            'online': bool(frame_id >= 0 and age is not None and age < 1.0),
            'width': int(meta.get('width', 0) or 0),
            'height': int(meta.get('height', 0) or 0),
            'pixel_format': str(meta.get('pixel_format_str', 'BGR')),
            'fps': round(fps, 1),
            'frame_version': int(frame_id),
            'frame_age': None if age is None else round(age, 3),
            'last_error': error,
        }

    def _select_output(self):
        from autolife_robot_sdk.utils import list_camera_shm_outputs

        outputs = list_camera_shm_outputs(
            self.robot_model,
            self.robot_version,
            module_name=self.module_name,
        )
        matches = [
            output for output in outputs
            if str(output.output_name).lower() == self.output_name.lower()
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f'expected one {self.module_name}/{self.output_name} camera '
                f'output, found {len(matches)}'
            )
        return matches[0]

    def _open_consumer(self):
        from autolife_robot_sdk.utils import open_camera_shm_consumer

        output = self._select_output()
        self._consumer = open_camera_shm_consumer(
            output, name='openarmx_306_v4_rgbd_head_color'
        )

    def _close_consumer(self):
        consumer = self._consumer
        self._consumer = None
        if consumer is not None:
            try:
                consumer.close()
            except Exception:
                pass

    def _set_error(self, message):
        with self._condition:
            self._last_error = str(message)
            self._condition.notify_all()

    def _run(self):
        import cv2

        minimum_interval = 1.0 / self.maximum_fps
        next_encode_time = None
        while not self._stop.is_set():
            try:
                if self._consumer is None:
                    self._open_consumer()
                item = self._consumer.get_latest(
                    nonblock=False, timeout=0.5, with_meta=True
                )
                if item is None:
                    continue
                frame, frame_id, meta = item
                now = time.monotonic()
                if next_encode_time is None:
                    next_encode_time = now
                # Keep a fixed cadence.  Setting the next deadline to
                # ``now + interval`` after every JPEG would add encoding time
                # to the period and turn a requested 30 FPS into ~20 FPS.
                if now + 0.001 < next_encode_time:
                    continue
                next_encode_time += minimum_interval
                if next_encode_time < now - minimum_interval:
                    next_encode_time = now + minimum_interval
                pixel_format = str(meta.get('pixel_format_str', '')).upper()
                if pixel_format not in ('BGR', 'RGB'):
                    raise RuntimeError(
                        f'unsupported RGB-D colour format: {pixel_format}'
                    )
                if pixel_format == 'RGB':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # The SDK frame may be a view into a two-slot shared-memory
                # ring.  Own one contiguous latest-frame copy so the producer
                # cannot overwrite pixels while the WebRTC encoder reads them.
                bgr = frame.copy(order='C')
                with self._condition:
                    jpeg_requested = now <= self._jpeg_demand_until
                jpeg = None
                if jpeg_requested:
                    ok, encoded = cv2.imencode(
                        '.jpg', bgr,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                    )
                    if not ok:
                        raise RuntimeError('OpenCV JPEG encoding failed')
                    jpeg = encoded.tobytes()
                with self._condition:
                    self._bgr = bgr
                    self._frame_id = int(frame_id)
                    if jpeg is not None:
                        self._jpeg = jpeg
                        self._jpeg_frame_id = int(frame_id)
                    self._meta = dict(meta)
                    self._last_frame_time = now
                    self._frame_times.append(now)
                    self._last_error = ''
                    self._condition.notify_all()
            except Exception as exc:
                self._set_error(exc)
                self._close_consumer()
                self._stop.wait(0.5)
