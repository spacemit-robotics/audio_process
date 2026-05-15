#!/usr/bin/env python3
"""
Multi-channel Sound Source Localization demo (Python)

Synthetic acceptance for MultiSoundLocator with a 3-mic equilateral array.
"""

import argparse
import math
import sys
from typing import List, Optional

try:
    import numpy as np
except ImportError:
    print("Error: numpy is required. pip install numpy")
    sys.exit(1)

try:
    from spacemit_audio_process import MultiSoundLocator, MicrophonePosition
except ImportError:
    try:
        from spacemit_audio_process._spacemit_audio_process import (
            MultiSoundLocator, MicrophonePosition
        )
    except ImportError as exc:
        print("Error: cannot import MultiSoundLocator")
        print("Install the package or set PYTHONPATH to the build directory.")
        print(f"Details: {exc}")
        sys.exit(1)


def parse_positions(spec: str) -> List["MicrophonePosition"]:
    """Parse 'x0,y0[,z0];x1,y1[,z1];...' into a list of MicrophonePosition."""
    positions = []
    for mic_spec in spec.split(";"):
        mic_spec = mic_spec.strip()
        if not mic_spec:
            continue
        parts = [p.strip() for p in mic_spec.split(",") if p.strip()]
        if len(parts) < 2:
            raise ValueError(
                f"each mic in --positions needs at least 'x,y' (got '{mic_spec}')"
            )
        vals = [float(p) for p in parts[:3]] + [0.0] * (3 - len(parts[:3]))
        positions.append(MicrophonePosition(vals[0], vals[1], vals[2]))
    if len(positions) < 2:
        raise ValueError("--positions needs at least 2 mics separated by ';'")
    return positions


LANCZOS_TAPS = 31
LANCZOS_HALF = LANCZOS_TAPS // 2
PASS = "PASS"
FAIL = "FAIL"


def normalize_angle(angle: float) -> float:
    value = math.fmod(angle, 360.0)
    if value < 0.0:
        value += 360.0
    return value


def circular_error(a: float, b: float) -> float:
    diff = abs(normalize_angle(a) - normalize_angle(b))
    return min(diff, 360.0 - diff)


def is_pair_endfire(angle: float) -> bool:
    return any(circular_error(angle, candidate) <= 0.5 for candidate in (60.0, 120.0, 240.0, 300.0))


def tolerance_for(angle: float, noise_enabled: bool) -> float:
    return 5.0 if noise_enabled or is_pair_endfire(angle) else 2.0


def sinc(x: float) -> float:
    if abs(x) < 1e-6:
        return 1.0
    pix = math.pi * x
    return math.sin(pix) / pix


def lanczos_kernel(x: float) -> float:
    a = float(LANCZOS_HALF)
    if abs(x) >= a:
        return 0.0
    return sinc(x) * sinc(x / a)


def make_chirp(n_samples: int, sample_rate: int) -> np.ndarray:
    duration = n_samples / float(sample_rate)
    f0 = 200.0
    f1 = 4000.0
    sweep = (f1 - f0) / duration
    t = np.arange(n_samples, dtype=np.float32) / float(sample_rate)
    phase = 2.0 * math.pi * (f0 * t + 0.5 * sweep * t * t)
    return (0.7 * np.sin(phase)).astype(np.float32)


def fractional_delay(base: np.ndarray, n_frames: int, delay_samples: float) -> np.ndarray:
    out = np.zeros(n_frames, dtype=np.float32)
    pad = (len(base) - n_frames) // 2
    for n in range(n_frames):
        src_pos = float(n + pad) - delay_samples
        center = math.floor(src_pos)
        value = 0.0
        weight_sum = 0.0
        for k in range(center - LANCZOS_HALF, center + LANCZOS_HALF + 1):
            if 0 <= k < len(base):
                x = src_pos - float(k)
                w = lanczos_kernel(x)
                value += float(base[k]) * w
                weight_sum += w
        if abs(weight_sum) > 1e-6:
            value /= weight_sum
        out[n] = value
    return out


def generate_synthetic(cfg, angle_deg: float, noise_snr_db: Optional[float]) -> np.ndarray:
    n_frames = cfg.frame_size * cfg.avg_frames
    pad = 64
    angle_rad = math.radians(angle_deg)
    direction = (math.cos(angle_rad), math.sin(angle_rad))
    base = make_chirp(n_frames + 2 * pad, cfg.sample_rate)

    channels = []
    for mic in cfg.microphones:
        advance_seconds = (mic.x * direction[0] + mic.y * direction[1]) / cfg.sound_speed
        delay_samples = -advance_seconds * cfg.sample_rate
        channels.append(fractional_delay(base, n_frames, delay_samples))

    data = np.stack(channels, axis=1).astype(np.float32)
    if noise_snr_db is not None:
        rng = np.random.default_rng(42)
        for ch in range(data.shape[1]):
            rms = float(np.sqrt(np.mean(data[:, ch] ** 2)))
            noise_std = rms / (10.0 ** (noise_snr_db / 20.0))
            data[:, ch] += rng.normal(0.0, noise_std, size=data.shape[0]).astype(np.float32)
    return data


def parse_sweep(spec: str) -> List[float]:
    parts = spec.split(":")
    if len(parts) != 3:
        raise ValueError("sweep must be start:end:step")
    start, end, step = (float(part) for part in parts)
    if abs(step) < 1e-6:
        raise ValueError("sweep step must be non-zero")
    values = []
    current = start
    if step > 0:
        while current <= end + 1e-4:
            values.append(normalize_angle(current))
            current += step
    else:
        while current >= end - 1e-4:
            values.append(normalize_angle(current))
            current += step
    return values


def default_angles() -> List[float]:
    angles = parse_sweep("0:330:30")
    angles.extend([45.0, 315.0, 60.0, 300.0])
    return angles


def run_test(args) -> int:
    if args.sweep:
        angles = parse_sweep(args.sweep)
    elif args.angle is not None:
        angles = [normalize_angle(args.angle)]
    else:
        angles = default_angles()

    print("=== Multi-channel Synthetic Accuracy Test ===")
    print(f"Geometry: equilateral | side: {args.mic_distance:.4f} m | sample rate: {args.rate} Hz")
    if args.noise_snr is not None:
        print(f"Noise SNR: {args.noise_snr:.1f} dB")
    print(f"azimuth_offset_deg: {args.azimuth_offset:.2f}\n")
    print(" expected |      est |    err |   conf | margin | pairs | status")

    passed = 0
    failed = 0
    noise_enabled = args.noise_snr is not None

    positions_override = None
    if args.positions:
        positions_override = parse_positions(args.positions)

    for angle in angles:
        if positions_override is not None:
            from spacemit_audio_process import MultiSoundLocatorConfig
            cfg = MultiSoundLocatorConfig()
            cfg.microphones = positions_override
        else:
            cfg = MultiSoundLocator.create_equilateral_triangle_config(args.mic_distance)
        cfg.sample_rate = args.rate
        cfg.frame_size = 512
        cfg.avg_frames = 4
        cfg.confidence_threshold = 0.1  # restored library default; margin_threshold (0.6 default) is the real silence gate
        cfg.azimuth_offset_deg = args.azimuth_offset
        cfg.max_avg_seconds = args.avg_seconds

        loc = MultiSoundLocator(cfg)
        if not loc.initialize():
            print("Initialize failed", file=sys.stderr)
            return 1

        data = generate_synthetic(cfg, angle, args.noise_snr)
        ready = loc.process(data)
        result = loc.result
        expected = normalize_angle(angle + args.azimuth_offset)
        err = circular_error(result.azimuth_deg, expected)
        tol = tolerance_for(angle, noise_enabled)
        ok = bool(ready and result.valid and err <= tol)
        if ok:
            passed += 1
        else:
            failed += 1

        print(
            f" {expected:8.2f} | {result.azimuth_deg:8.2f} | {err:6.2f} | "
            f"{result.confidence:6.3f} | {result.score_margin:6.3f} | "
            f"{result.valid_pairs:5d} | {PASS if ok else FAIL}"
        )

    print(f"\n--- Results: {passed}/{passed + failed} passed ---")
    return 0 if failed == 0 else 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-channel SSL synthetic demo")
    parser.add_argument("-t", "--test", action="store_true", help="Synthetic accuracy test")
    parser.add_argument("-d", "--mic-distance", type=float, default=0.063,
                        help="Equilateral side length in meters (default: 0.063)")
    parser.add_argument("-r", "--rate", type=int, default=16000,
                        help="Sample rate in Hz (default: 16000)")
    parser.add_argument("--angle", type=float, help="Single synthetic angle")
    parser.add_argument("--sweep", help="Synthetic sweep start:end:step")
    parser.add_argument("--noise-snr", type=float, help="Add Gaussian noise at this SNR in dB")
    parser.add_argument("--azimuth-offset", type=float, default=0.0,
                        help="Array-frame to robot-frame offset in degrees")
    parser.add_argument("--avg-seconds", type=float, default=0.0,
                        help="Sliding-window length for GetAverageAzimuth (s, default 0 = unbounded since Reset)")
    parser.add_argument("--positions", type=str, default=None,
                        help="Override mic positions, e.g. '0.036,0;-0.018,0.0315;-0.018,-0.0315' (meters); skips equilateral default")
    args = parser.parse_args()

    if not args.test:
        parser.print_usage()
        sys.exit(1)
    sys.exit(run_test(args))


if __name__ == "__main__":
    main()
