"""Small, nonblocking feedback events shared by recorder and DAgger controls."""

import time
import uuid

# Distinct rhythms in milliseconds. Browser scheduling never blocks ROS or IK.
PATTERNS = {
    "countdown": [35], "start": [80, 80], "mark": [50, 50, 50],
    "saving": [45, 140], "save": [220], "discarding": [140, 45],
    "discard": [160, 160], "resetting": [90, 90, 250],
    "reset": [250, 90], "takeover": [300, 70],
    "cancel": [70, 180, 70], "error": [240, 240, 240],
}


def feedback_packet(event):
    return {"id": uuid.uuid4().hex, "event": event, "wall_time": time.time(),
            "pulses_ms": PATTERNS[event], "intensity": 0.65}
