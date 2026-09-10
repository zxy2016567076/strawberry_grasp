"""Float MLP using the existing C header weights; never retrain in the demo."""
from collections import deque
from pathlib import Path
import hashlib
import re
import numpy as np

WEIGHTS = Path(__file__).resolve().parents[1] / "mcu" / "tinyml_weights.h"
CLASSES = ["STABLE", "SLIP_RISK", "OVERFORCE"]


def prepare_input(deltas):
    values = np.asarray(deltas, dtype=np.float32)
    if values.ndim != 1 or not np.isfinite(values).all() or np.any(values < 0) or np.any(values > 65535):
        raise ValueError("expected uint16-range FSR deltas")
    result = np.zeros(16, dtype=np.float32)
    values = values[-16:]
    if len(values):
        result[-len(values):] = values / np.float32(1000)
    return result


def load_weights(path=WEIGHTS):
    text = Path(path).read_text(encoding="utf-8")
    arrays = []
    for name, shape in [("w1", (16, 8)), ("b1", (8,)), ("w2", (8, 4)),
                        ("b2", (4,)), ("w3", (4, 3)), ("b3", (3,))]:
        match = re.search(r"tinyml_" + name + r"\[[^;=]+?=\s*(\{.*?\});", text, re.S)
        if not match:
            raise ValueError(f"missing weight {name}")
        numbers = re.findall(r"([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)f", match[1])
        arrays.append(np.array(numbers, dtype=np.float32).reshape(shape))
    return arrays


def classify(deltas):
    x = prepare_input(deltas)
    w1, b1, w2, b2, w3, b3 = load_weights()
    logits = np.maximum(0, np.maximum(0, x @ w1 + b1) @ w2 + b2) @ w3 + b3
    return {"class": CLASSES[int(np.argmax(logits))], "logits": logits.tolist(),
            "input": x.tolist(), "weights_sha256": hashlib.sha256(WEIGHTS.read_bytes()).hexdigest(),
            "note": "float MLP diagnostic only; logits are not probabilities"}


class Sampler:
    """Independent periodic task. Timestamp progression is virtual, not RA6M5 timing."""
    def __init__(self, period_ms=5):
        if type(period_ms) is not int or period_ms < 1:
            raise ValueError("sampling period must be a positive integer millisecond")
        self.period_ms = period_ms
        self.next_ms = 0
        self.samples = deque(maxlen=4096)
        self.count = 0

    def tick(self, now_ms, source):
        if now_ms >= self.next_ms:
            value = int(source(now_ms))
            if not 0 <= value <= 65535:
                raise ValueError("invalid pressure delta")
            self.samples.append((now_ms, value))
            self.count += 1
            # No fabricated backfilled samples when a real scheduler misses deadlines.
            self.next_ms = now_ms + self.period_ms
            return value
        return None
