#!/usr/bin/env python3
"""Convert square MOV animations into the ESP32 screen WVJ stream format."""

from __future__ import annotations

import argparse
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path


MAGIC_WVJ1 = 0x314A5657
HEADER_SIZE = 32
FRAME_HEADER_SIZE = 8
DEFAULT_MAX_FRAME_BYTES = 220 * 1024


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run_ffmpeg(input_path: Path, frame_dir: Path, fps: int, quality: int, size: int) -> None:
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg not found. Install ffmpeg before converting screen videos.")

    frame_pattern = frame_dir / "%06d.jpg"
    vf = (
        f"fps={fps},"
        f"scale={size}:{size}:force_original_aspect_ratio=increase:flags=lanczos,"
        f"crop={size}:{size},"
        "format=yuvj420p"
    )
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(input_path),
        "-vf",
        vf,
        "-vcodec",
        "mjpeg",
        "-q:v",
        str(quality),
        str(frame_pattern),
    ]
    subprocess.run(cmd, check=True)


def write_wvj(
    frames: list[Path],
    output_path: Path,
    fps: int,
    size: int,
    max_frame_bytes: int,
) -> tuple[int, int, int]:
    if not frames:
        raise SystemExit(f"No frames were generated for {output_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    duration_us = round(1_000_000 / fps)
    max_size = 0
    payload_size = 0

    header = struct.pack(
        "<IHHHHHHIIII",
        MAGIC_WVJ1,
        1,
        HEADER_SIZE,
        size,
        size,
        fps,
        0,
        len(frames),
        0,
        HEADER_SIZE,
        0,
    )

    with output_path.open("wb") as out:
        out.write(header)
        for frame in frames:
            data = frame.read_bytes()
            if not data.startswith(b"\xff\xd8"):
                raise SystemExit(f"{frame} is not a JPEG frame")
            if len(data) > max_frame_bytes:
                raise SystemExit(
                    f"{frame.name} is {len(data)} bytes, above max {max_frame_bytes}. "
                    "Raise --quality value or lower --fps/--size."
                )
            out.write(struct.pack("<II", len(data), duration_us))
            out.write(data)
            max_size = max(max_size, len(data))
            payload_size += FRAME_HEADER_SIZE + len(data)

    return len(frames), max_size, HEADER_SIZE + payload_size


def convert_one(
    label: str,
    input_path: Path,
    output_path: Path,
    fps: int,
    quality: int,
    size: int,
    max_frame_bytes: int,
    keep_frames: bool,
) -> None:
    if not input_path.exists():
        raise SystemExit(f"Missing input video: {input_path}")

    with tempfile.TemporaryDirectory(prefix=f"screen_{label}_") as tmp:
        frame_dir = Path(tmp) / "frames"
        frame_dir.mkdir()
        run_ffmpeg(input_path, frame_dir, fps, quality, size)
        frames = sorted(frame_dir.glob("*.jpg"))
        frame_count, max_jpeg, total_size = write_wvj(
            frames,
            output_path,
            fps,
            size,
            max_frame_bytes,
        )
        print(
            f"{label}: {frame_count} frames, {total_size / (1024 * 1024):.2f} MiB, "
            f"max frame {max_jpeg / 1024:.1f} KiB -> {output_path}"
        )

        if keep_frames:
            kept_dir = output_path.with_suffix("")
            if kept_dir.exists():
                shutil.rmtree(kept_dir)
            shutil.copytree(frame_dir, kept_dir)


def parse_args() -> argparse.Namespace:
    root = default_project_root()
    return argparse.ArgumentParser(description=__doc__).parse_args()


def main() -> None:
    root = default_project_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--boot",
        type=Path,
        default=root / "video" / "\u5f00\u673a\u52a8\u753b.mov",
        help="Boot animation MOV path.",
    )
    parser.add_argument(
        "--loop",
        type=Path,
        default=root / "video" / "\u5faa\u73af\u52a8\u753b.mov",
        help="Loop animation MOV path.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=root / "data" / "video",
        help="Output directory for boot.wvj and loop.wvj.",
    )
    parser.add_argument("--fps", type=int, default=15, help="Output frame rate.")
    parser.add_argument("--quality", type=int, default=8, help="ffmpeg MJPEG q:v value; higher is smaller.")
    parser.add_argument("--size", type=int, default=360, help="Square output size in pixels.")
    parser.add_argument("--max-frame-bytes", type=int, default=DEFAULT_MAX_FRAME_BYTES)
    parser.add_argument("--keep-frames", action="store_true", help="Keep extracted JPEG frames beside output.")
    args = parser.parse_args()

    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    if args.size <= 0 or args.size > 4096:
        raise SystemExit("--size is out of range")
    if not 2 <= args.quality <= 31:
        raise SystemExit("--quality must be between 2 and 31")

    convert_one(
        "boot",
        args.boot,
        args.out_dir / "boot.wvj",
        args.fps,
        args.quality,
        args.size,
        args.max_frame_bytes,
        args.keep_frames,
    )
    convert_one(
        "loop",
        args.loop,
        args.out_dir / "loop.wvj",
        args.fps,
        args.quality,
        args.size,
        args.max_frame_bytes,
        args.keep_frames,
    )


if __name__ == "__main__":
    main()
