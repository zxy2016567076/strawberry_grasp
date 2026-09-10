"""Simulation-only UART byte framing. Deliberately no serial-port backend.

    '~' + lowercase hex(UTF-8 JSON) + ':' + lowercase CRC32 hex + LF.
    The wire alphabet excludes ALL board-firmware uppercase command letters.
"""
import json
import zlib

MODEL = "six-motion-plus-gripper"
MAX_FRAME = 2048


def encode(message):
    raw = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
    frame = b"~" + raw.hex().encode("ascii") + b":" + f"{zlib.crc32(raw):08x}".encode() + b"\n"
    if len(frame) > MAX_FRAME:
        raise ValueError("frame too long")
    return frame


def decode(frame):
    if len(frame) > MAX_FRAME or not frame.startswith(b"~") or not frame.endswith(b"\n"):
        raise ValueError("not a V3 frame")
    try:
        payload, checksum = frame[1:-1].split(b":")
        if any(c not in b"0123456789abcdef" for c in payload + checksum) or len(checksum) != 8:
            raise ValueError("invalid frame alphabet")
        raw = bytes.fromhex(payload.decode("ascii"))
        if zlib.crc32(raw) != int(checksum, 16):
            raise ValueError("CRC mismatch")
        data = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
    except (UnicodeError, TypeError) as exc:
        raise ValueError("invalid frame") from exc
    if not isinstance(data, dict) or data.get("v") != 3 or data.get("model") != MODEL:
        raise ValueError("wrong version or motion-axis model")
    return data


def message(seq, op, **fields):
    return {"v": 3, "model": MODEL, "seq": seq, "op": op, **fields}


class FrameStream:
    """Bounded line receiver supporting UART fragmentation and multiple frames."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        frames = []
        for byte in chunk:
            self.buffer.append(byte)
            if len(self.buffer) > MAX_FRAME:
                self.buffer.clear()
                raise ValueError("UART receive overflow")
            if byte == 10:
                frames.append(bytes(self.buffer))
                self.buffer.clear()
        return frames
