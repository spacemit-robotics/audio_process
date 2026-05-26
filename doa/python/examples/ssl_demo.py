#!/usr/bin/env python3
"""Unified Sound Source Localization demo (Python; mirrors C++ ssl_demo).

Channel count is selected with ``-c N``:
  - ``-c 2``          → SoundLocator (2-ch GCC-PHAT, DOA ∈ [0°, 180°])
  - ``-c 3+``         → MultiSoundLocator (3+ ch planar MPCC-LSQ,
                         azimuth ∈ [0°, 360°))
  - ``-f WAV`` 无 ``-c``: 自动按 WAV 头判定。

Examples::

    ssl_demo.py -c 2 -t -d 0.058
    ssl_demo.py -c 2 -f stereo.wav -d 0.058 --flip
    ssl_demo.py -c 3 -t -d 0.063 --sweep 0:330:30
    ssl_demo.py -c 3 -f 3ch.wav -d 0.063 -v
    ssl_demo.py -c 3 -t -d 0.063 --quality-threshold 0.5
"""

import argparse
import math
import sys
import time
import wave
from typing import List, Optional

try:
    import numpy as np
except ImportError:
    print("Error: numpy is required. pip install numpy")
    sys.exit(1)

try:
    from spacemit_audio_process import (
        MicrophonePosition,
        MultiSoundLocator,
        MultiSoundLocatorConfig,
        SoundLocator,
        SoundLocatorConfig,
    )
except ImportError as exc:
    print("Error: cannot import spacemit_audio_process")
    print("Install the package or set PYTHONPATH to the build directory:")
    print("  export PYTHONPATH=/path/to/doa/build")
    print(f"\nDetails: {exc}")
    sys.exit(1)


DEFAULT_MIC_DISTANCE_2CH = 0.058
DEFAULT_MIC_DISTANCE_3CH = 0.063
LANCZOS_TAPS = 31
LANCZOS_HALF = LANCZOS_TAPS // 2


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def normalize_angle(angle: float) -> float:
    value = math.fmod(angle, 360.0)
    if value < 0.0:
        value += 360.0
    if value >= 360.0:
        value -= 360.0
    return value


def circular_error(a: float, b: float) -> float:
    diff = abs(normalize_angle(a) - normalize_angle(b))
    return min(diff, 360.0 - diff)


def warn_default_mic_distance(user_set: bool, used: float, mode_tag: str) -> None:
    if not user_set:
        print(
            f"[ssl_demo] WARNING: -d / --mic-distance not set; using demo "
            f"default {used:.4f} m for {mode_tag}. Production code MUST "
            f"measure and set the real physical mic spacing — endfire "
            f"angles fail silently otherwise.",
            file=sys.stderr,
        )


# ---------------------------------------------------------------------------
# 2-channel mode (SoundLocator)
# ---------------------------------------------------------------------------

def report_angle_2ch(doa: float, flip: bool) -> float:
    angle = 180.0 - doa if flip else doa
    return max(0.0, min(180.0, angle))


def run_test_2ch(mic_dist: float, sample_rate: int, flip: bool) -> int:
    print("=== 2-ch Synthetic Accuracy Test ===")
    print(f"Mic distance: {mic_dist:.4f} m | Sample rate: {sample_rate} Hz\n")
    if flip:
        print("Angle mapping: flipped (reported = 180 - raw DOA)\n")

    cfg = SoundLocatorConfig()
    cfg.sample_rate = sample_rate
    cfg.mic_distance = mic_dist
    cfg.frame_size = 512
    cfg.avg_frames = 4
    cfg.confidence_threshold = 0.05

    loc = SoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1
    print(f"Max delay: {loc.max_delay_samples} samples\n")

    c = cfg.sound_speed
    fs = float(sample_rate)
    total_samples = cfg.frame_size * cfg.avg_frames
    sinc_half = 16

    test_angles = [90, 85, 80, 75, 70, 60, 45, 30, 15, 0,
                   95, 100, 120, 135, 150, 180]

    passed = 0
    failed = 0
    rng = np.random.default_rng(42)

    for angle in test_angles:
        loc.reset()
        phys_angle = 90.0 - angle
        theta_rad = math.radians(phys_angle)
        delay_sec = mic_dist * math.sin(theta_rad) / c
        delay_samples = -delay_sec * fs

        ch0_padded = (rng.standard_normal(total_samples + 2 * sinc_half)
                      .astype(np.float32) * 0.5)
        ch0 = ch0_padded[sinc_half:sinc_half + total_samples].copy()

        ch1 = np.zeros(total_samples, dtype=np.float32)
        for i in range(total_samples):
            src_pos = float(i + sinc_half) - delay_samples
            src_int = int(math.floor(src_pos))
            val = 0.0
            for k in range(src_int - sinc_half + 1, src_int + sinc_half + 1):
                if 0 <= k < len(ch0_padded):
                    x = src_pos - float(k)
                    if abs(x) < 1e-6:
                        sinc = 1.0
                    else:
                        sinc = math.sin(math.pi * x) / (math.pi * x)
                    w = 0.5 * (1.0 + math.cos(math.pi * x / sinc_half))
                    val += float(ch0_padded[k]) * sinc * w
            ch1[i] = val

        ready = False
        for offset in range(0, total_samples, cfg.frame_size):
            end = offset + cfg.frame_size
            if loc.process_separate(ch0[offset:end], ch1[offset:end]):
                ready = True
        if not ready:
            print(f"  {report_angle_2ch(angle, flip):6.1f}° → NO RESULT")
            failed += 1
            continue

        expected = report_angle_2ch(angle, flip)
        est = report_angle_2ch(loc.doa, flip)
        err_abs = abs(est - expected)
        if 1.0 < expected < 179.0:
            err_pct = err_abs / min(expected, 180.0 - expected) * 100.0
        else:
            err_pct = err_abs
        status = "PASS" if (err_pct < 3.0 or err_abs < 2.5) else "FAIL"
        if status == "PASS":
            passed += 1
        else:
            failed += 1
        print(
            f"  {expected:6.1f}° → {est:7.2f}° | err: {err_abs:.2f}° "
            f"({err_pct:.1f}%) | conf: {loc.confidence:.3f} | "
            f"delay: {delay_samples:.3f} samp | {status}"
        )

    print(f"\n--- Results: {passed}/{passed + failed} passed ---")
    return 1 if failed > 0 else 0


def run_file_2ch(path: str, mic_dist: float, verbose: bool, flip: bool) -> int:
    try:
        wf = wave.open(path, "rb")
    except Exception as exc:
        print(f"Cannot open {path}: {exc}", file=sys.stderr)
        return 1
    channels = wf.getnchannels()
    sample_rate = wf.getframerate()
    n_frames = wf.getnframes()
    sampwidth = wf.getsampwidth()
    if channels != 2:
        print(
            f"WAV must be stereo for -c 2 (got {channels} channels). "
            f"For multi-channel WAV use -c 3.",
            file=sys.stderr,
        )
        wf.close()
        return 1
    if sampwidth != 2:
        print(f"Only PCM16 WAV supported (got {sampwidth * 8}-bit)",
              file=sys.stderr)
        wf.close()
        return 1
    if n_frames == 0:
        print("WAV contains no audio frames", file=sys.stderr)
        wf.close()
        return 1
    raw = wf.readframes(n_frames)
    wf.close()
    samples = np.frombuffer(raw, dtype=np.int16)

    duration = n_frames / sample_rate
    print(f"=== 2-ch WAV File: {path} ===")
    print(
        f"Sample rate: {sample_rate} Hz | Samples: {n_frames} | "
        f"Duration: {duration:.2f} s"
    )
    if flip:
        print("Angle mapping: flipped")

    cfg = SoundLocatorConfig()
    cfg.sample_rate = sample_rate
    cfg.mic_distance = mic_dist
    cfg.frame_size = 512
    cfg.avg_frames = 4

    loc = SoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    offset = 0
    frame_no = 0
    t_start = time.monotonic()
    while offset < n_frames:
        chunk = min(cfg.frame_size, n_frames - offset)
        interleaved = samples[offset * 2:(offset + chunk) * 2]
        if loc.process_int16(interleaved):
            frame_no += 1
            if verbose:
                if loc.is_valid:
                    max_tdoa = cfg.mic_distance / cfg.sound_speed
                    delay_ratio = abs(loc.tdoa) / max_tdoa
                    print(
                        f"[Frame {frame_no:03d}] DOA: "
                        f"{report_angle_2ch(loc.doa, flip):7.2f}° | "
                        f"TDOA: {loc.tdoa:+.6f}s | Ratio: {delay_ratio:.3f} "
                        f"| Conf: {loc.confidence:.3f}"
                    )
                else:
                    print(
                        f"[Frame {frame_no:03d}] DOA: ------- | "
                        f"Conf: {loc.confidence:.3f} (below threshold)"
                    )
        offset += chunk
    t_end = time.monotonic()
    process_ms = (t_end - t_start) * 1000
    rtf = (process_ms / 1000) / duration
    if loc.result_count > 0:
        print(f"\nest angle = {report_angle_2ch(loc.average_doa, flip):.1f}")
    else:
        print("\nest angle = N/A (no valid frames)")
    print(f"Process: {process_ms:.1f} ms | Audio: {duration:.2f} s | "
          f"RTF: {rtf:.4f}")
    return 0


def run_live_2ch(mic_dist: float, sample_rate: int, device_index: int,
                 verbose: bool, flip: bool) -> int:
    try:
        import spacemit_audio
    except ImportError:
        print("Error: spacemit_audio is required for live capture.")
        return 1

    print("=== 2-ch Live Capture ===")
    cfg = SoundLocatorConfig()
    cfg.sample_rate = sample_rate
    cfg.mic_distance = mic_dist
    cfg.frame_size = 512
    cfg.avg_frames = 4
    print(
        f"Mic distance: {mic_dist:.4f} m | Sample rate: {sample_rate} Hz | "
        f"Device: {device_index}"
    )

    loc = SoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    chunk_size = cfg.frame_size * 2 * 2
    spacemit_audio.init(sample_rate=sample_rate, channels=2,
                        chunk_size=chunk_size, capture_device=device_index,
                        player_device=-1)
    capture = spacemit_audio.AudioCapture(device_index)

    state = {"frame_no": 0, "callbacks": 0, "frames": 0, "valid": 0,
             "confidence": 0.0}

    def callback(data: bytes) -> None:
        state["callbacks"] += 1
        samples = np.frombuffer(data, dtype=np.int16)
        frames = len(samples) // 2
        if frames <= 0:
            return
        samples = samples[:frames * 2]
        if loc.process_int16(samples):
            state["frames"] += 1
            state["frame_no"] += 1
            state["confidence"] = loc.confidence
            if loc.is_valid:
                state["valid"] += 1
                print(
                    f"\r[{state['frame_no']:04d}] DOA: "
                    f"{report_angle_2ch(loc.doa, flip):7.2f}° | "
                    f"Conf: {loc.confidence:.3f}    ",
                    end="", flush=True,
                )
            elif verbose:
                print(
                    f"\r[{state['frame_no']:04d}] DOA: ------- | "
                    f"Conf: {loc.confidence:.3f}    ",
                    end="", flush=True,
                )

    capture.set_callback(callback)
    if not capture.start(sample_rate=sample_rate, channels=2,
                         chunk_size=chunk_size):
        print("Failed to start capture", file=sys.stderr)
        capture.close()
        return 1
    print("Press Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        capture.close()
    print(
        f"\nStopped. callbacks: {state['callbacks']}, "
        f"frames: {state['frames']}, valid: {state['valid']}"
    )
    return 0


# ---------------------------------------------------------------------------
# 3+ channel mode (MultiSoundLocator)
# ---------------------------------------------------------------------------

def is_pair_endfire(angle: float) -> bool:
    return any(
        circular_error(angle, c) <= 0.5
        for c in (60.0, 120.0, 240.0, 300.0)
    )


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


def fractional_delay(base: np.ndarray, n_frames: int,
                     delay_samples: float) -> np.ndarray:
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


def generate_synthetic(cfg, angle_deg: float,
                       noise_snr_db: Optional[float]) -> np.ndarray:
    n_frames = cfg.frame_size * cfg.avg_frames
    pad = 64
    angle_rad = math.radians(angle_deg)
    direction = (math.cos(angle_rad), math.sin(angle_rad))
    base = make_chirp(n_frames + 2 * pad, cfg.sample_rate)
    channels = []
    for mic in cfg.microphones:
        advance = (mic.x * direction[0]
                   + mic.y * direction[1]) / cfg.sound_speed
        delay = -advance * cfg.sample_rate
        channels.append(fractional_delay(base, n_frames, delay))
    data = np.stack(channels, axis=1).astype(np.float32)
    if noise_snr_db is not None:
        rng = np.random.default_rng(42)
        for ch in range(data.shape[1]):
            rms = float(np.sqrt(np.mean(data[:, ch] ** 2)))
            std = rms / (10.0 ** (noise_snr_db / 20.0))
            data[:, ch] += rng.normal(
                0.0, std, size=data.shape[0]
            ).astype(np.float32)
    return data


def parse_sweep(spec: str) -> List[float]:
    parts = spec.split(":")
    if len(parts) != 3:
        raise ValueError("sweep must be start:end:step")
    start, end, step = (float(p) for p in parts)
    if abs(step) < 1e-6:
        raise ValueError("sweep step must be non-zero")
    values: List[float] = []
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


def parse_positions(spec: str) -> List["MicrophonePosition"]:
    positions = []
    for mic_spec in spec.split(";"):
        mic_spec = mic_spec.strip()
        if not mic_spec:
            continue
        parts = [p.strip() for p in mic_spec.split(",") if p.strip()]
        if len(parts) < 2:
            raise ValueError(
                f"each mic in --positions needs at least 'x,y' "
                f"(got '{mic_spec}')"
            )
        vals = ([float(p) for p in parts[:3]]
                + [0.0] * (3 - len(parts[:3])))
        positions.append(
            MicrophonePosition(vals[0], vals[1], vals[2])
        )
    if len(positions) < 2:
        raise ValueError(
            "--positions needs at least 2 mics separated by ';'"
        )
    return positions


def make_multi_cfg(args, sample_rate: int) -> MultiSoundLocatorConfig:
    if args.positions:
        cfg = MultiSoundLocatorConfig()
        cfg.microphones = parse_positions(args.positions)
    else:
        cfg = MultiSoundLocator.create_equilateral_triangle_config(
            args.mic_distance
        )
    cfg.sample_rate = sample_rate
    cfg.frame_size = 512
    cfg.avg_frames = 4
    cfg.confidence_threshold = 0.1
    cfg.azimuth_offset_deg = args.azimuth_offset
    cfg.max_avg_seconds = args.avg_seconds
    if args.quality_threshold is not None:
        cfg.quality_threshold = args.quality_threshold
    if args.margin_threshold is not None:
        cfg.margin_threshold = args.margin_threshold
    if args.closure_threshold_samples is not None:
        cfg.closure_threshold_samples = args.closure_threshold_samples
    if args.closure_threshold_fraction is not None:
        cfg.closure_threshold_fraction = args.closure_threshold_fraction
    if args.max_frequency_hz is not None:
        cfg.max_frequency_hz = args.max_frequency_hz
    return cfg


def run_test_multi(args) -> int:
    if args.sweep:
        angles = parse_sweep(args.sweep)
    elif args.angle is not None:
        angles = [normalize_angle(args.angle)]
    else:
        angles = parse_sweep("0:330:30")

    print("=== 3+ch Synthetic Accuracy Test ===")
    print(
        f"Geometry: equilateral | side: {args.mic_distance:.4f} m | "
        f"sample rate: {args.rate} Hz"
    )
    if args.noise_snr is not None:
        print(f"Noise SNR: {args.noise_snr:.1f} dB")
    print(f"azimuth_offset_deg: {args.azimuth_offset:.2f}\n")
    print(
        " expected |      est |    err |   conf |  qual | margin | "
        "clos.us | pairs | status"
    )

    passed = 0
    failed = 0
    noise_enabled = args.noise_snr is not None

    for angle in angles:
        cfg = make_multi_cfg(args, args.rate)
        loc = MultiSoundLocator(cfg)
        if not loc.initialize():
            print("Initialize failed", file=sys.stderr)
            return 1
        data = generate_synthetic(cfg, angle, args.noise_snr)
        ready = loc.process(data)
        r = loc.result
        expected = normalize_angle(angle + args.azimuth_offset)
        err = circular_error(r.azimuth_deg, expected)
        tol = tolerance_for(angle, noise_enabled)
        ok = bool(ready and r.valid and err <= tol)
        if ok:
            passed += 1
        else:
            failed += 1
        print(
            f" {expected:8.2f} | {r.azimuth_deg:8.2f} | {err:6.2f} | "
            f"{r.confidence:6.3f} | {r.quality:5.3f} | {r.score_margin:6.3f} "
            f"| {r.closure_residual_sec * 1e6:7.2f} | "
            f"{r.valid_pairs:5d} | {'PASS' if ok else 'FAIL'}"
        )
    print(f"\n--- Results: {passed}/{passed + failed} passed ---")
    return 0 if failed == 0 else 1


def run_file_multi(path: str, args) -> int:
    try:
        wf = wave.open(path, "rb")
    except Exception as exc:
        print(f"Cannot open {path}: {exc}", file=sys.stderr)
        return 1
    channels = wf.getnchannels()
    sample_rate = wf.getframerate()
    n_frames = wf.getnframes()
    sampwidth = wf.getsampwidth()
    if channels < 3:
        print(
            f"WAV must have >= 3 channels for -c 3+ (got {channels}). "
            f"For stereo use -c 2.",
            file=sys.stderr,
        )
        wf.close()
        return 1
    if sampwidth != 2:
        print(f"Only PCM16 WAV supported (got {sampwidth * 8}-bit)",
              file=sys.stderr)
        wf.close()
        return 1
    if n_frames == 0:
        print("WAV contains no audio frames", file=sys.stderr)
        wf.close()
        return 1
    raw = wf.readframes(n_frames)
    wf.close()
    samples = np.frombuffer(raw, dtype=np.int16).reshape(-1, channels)

    cfg = make_multi_cfg(args, sample_rate)
    if len(cfg.microphones) != channels:
        print(
            f"WAV has {channels} channels but config has "
            f"{len(cfg.microphones)} mic positions; pass --positions or "
            f"use a matching equilateral side.",
            file=sys.stderr,
        )
        return 1

    loc = MultiSoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    duration = n_frames / sample_rate
    print(f"=== {channels}-ch WAV File: {path} ===")
    print(
        f"Sample rate: {sample_rate} Hz | Frames: {n_frames} | "
        f"Duration: {duration:.2f} s"
    )

    offset = 0
    frame_no = 0
    t_start = time.monotonic()
    while offset < n_frames:
        chunk = min(cfg.frame_size, n_frames - offset)
        block = samples[offset:offset + chunk]
        if loc.process_int16(block):
            frame_no += 1
            if args.verbose:
                r = loc.result
                if r.valid:
                    print(
                        f"[Frame {frame_no:03d}] az: {r.azimuth_deg:7.2f}° "
                        f"| conf: {r.confidence:.3f} | qual: {r.quality:.3f} "
                        f"| clos: {r.closure_residual_sec * 1e6:.2f} us | "
                        f"pairs: {r.valid_pairs}"
                    )
                else:
                    print(
                        f"[Frame {frame_no:03d}] az: ------- | "
                        f"conf: {r.confidence:.3f}"
                    )
        offset += chunk
    t_end = time.monotonic()
    process_ms = (t_end - t_start) * 1000
    rtf = (process_ms / 1000) / duration
    if loc.result_count > 0:
        print(
            f"\nest azimuth = {loc.average_azimuth:.1f} "
            f"(R = {loc.average_resultant_length:.3f})"
        )
    else:
        print("\nest azimuth = N/A (no valid frames)")
    print(f"Process: {process_ms:.1f} ms | Audio: {duration:.2f} s | "
          f"RTF: {rtf:.4f}")
    return 0


def run_live_multi(args) -> int:
    try:
        import spacemit_audio
    except ImportError:
        print("Error: spacemit_audio is required for live capture.")
        return 1

    cfg = make_multi_cfg(args, args.rate)
    n_ch = len(cfg.microphones)
    loc = MultiSoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    chunk_size = cfg.frame_size * n_ch * 2
    spacemit_audio.init(sample_rate=args.rate, channels=n_ch,
                        chunk_size=chunk_size,
                        capture_device=args.input_device, player_device=-1)
    capture = spacemit_audio.AudioCapture(args.input_device)
    state = {"frames": 0, "valid": 0}

    def callback(data: bytes) -> None:
        samples = np.frombuffer(data, dtype=np.int16).reshape(-1, n_ch)
        if samples.shape[0] == 0:
            return
        if loc.process_int16(samples):
            state["frames"] += 1
            r = loc.result
            if r.valid:
                state["valid"] += 1
                print(
                    f"\r[{state['frames']:04d}] az: {r.azimuth_deg:7.2f}° "
                    f"| conf: {r.confidence:.3f} | qual: {r.quality:.3f} "
                    f"| clos: {r.closure_residual_sec * 1e6:.2f} us    ",
                    end="", flush=True,
                )
            elif args.verbose:
                print(
                    f"\r[{state['frames']:04d}] az: ------- | "
                    f"conf: {r.confidence:.3f}    ",
                    end="", flush=True,
                )

    capture.set_callback(callback)
    if not capture.start(sample_rate=args.rate, channels=n_ch,
                         chunk_size=chunk_size):
        print("Failed to start capture", file=sys.stderr)
        capture.close()
        return 1
    print(
        f"=== Live {n_ch}-ch Capture ===\n"
        f"side: {args.mic_distance:.4f} m | rate: {args.rate} Hz | "
        f"device: {args.input_device}\nPress Ctrl+C to stop.\n"
    )
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        capture.close()
    print(f"\nStopped. frames: {state['frames']}, valid: {state['valid']}")
    return 0


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=("Unified SSL demo (mirrors C++ ssl_demo). "
                     "Channel count selected with -c N."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("-c", "--channels", type=int, default=0,
                   help="2 → SoundLocator; >=3 → MultiSoundLocator. "
                        "0 (default) auto-detects from WAV in -f mode, "
                        "else 2.")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("-t", "--test", action="store_true",
                      help="Synthetic accuracy test")
    mode.add_argument("-f", "--file", metavar="WAV",
                      help="Process a PCM16 WAV file")
    mode.add_argument("-l", "--live", action="store_true",
                      help="Live capture from microphone")

    p.add_argument("-d", "--mic-distance", type=float, default=None,
                   help=(f"Mic spacing m. Default {DEFAULT_MIC_DISTANCE_2CH} "
                         f"(2-ch) / {DEFAULT_MIC_DISTANCE_3CH} (3-ch); "
                         "warning if not set."))
    p.add_argument("-r", "--rate", type=int, default=16000,
                   help="Sample rate Hz (default 16000)")
    p.add_argument("-i", "--input-device", type=int, default=-1,
                   help="Capture device index (default -1 = auto)")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="Per-frame output (file/live mode)")

    # 2-ch specific
    p.add_argument("--flip", action="store_true",
                   help=("Report 180 - DOA (legacy 2-ch alias). For -c 3 "
                         "use --azimuth-offset instead."))

    # 3+ ch specific
    p.add_argument("--angle", type=float, default=None,
                   help="Single synthetic angle (test mode, -c 3+)")
    p.add_argument("--sweep", default=None,
                   help="Synthetic sweep A:B:S (test mode, -c 3+)")
    p.add_argument("--noise-snr", type=float, default=None,
                   help="Gaussian noise SNR in dB (-c 3+)")
    p.add_argument("--azimuth-offset", type=float, default=0.0,
                   help="Array → robot frame offset deg (-c 3+)")
    p.add_argument("--positions", default=None,
                   help=("Override mic positions, e.g. "
                         "'x0,y0;x1,y1;x2,y2' (meters; -c 3+)"))
    p.add_argument("--avg-seconds", type=float, default=0.0,
                   help=("Sliding-window length for GetAverageAzimuth "
                         "(s; 0 = unbounded; set positive for live)"))
    p.add_argument("--quality-threshold", type=float, default=None,
                   help="Override config.quality_threshold (default 0.0)")
    p.add_argument("--margin-threshold", type=float, default=None,
                   help="Override config.margin_threshold (default 0.6)")
    p.add_argument("--closure-threshold-samples", type=float, default=None,
                   help=("Override config.closure_threshold_samples "
                         "(N=3 only; default 0.0; effective=max(samples, "
                         "fraction*physical_max))"))
    p.add_argument("--closure-threshold-fraction", type=float, default=None,
                   help=("Override config.closure_threshold_fraction "
                         "(N=3 only; default 0.3; set both closure "
                         "thresholds <=0 to disable gate)"))
    p.add_argument("--max-frequency-hz", type=float, default=None,
                   help=("Override config.max_frequency_hz (default 0 "
                         "= auto alias-safe c / 2·d_max)"))
    return p


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    # Auto-detect channels for -f mode if -c not set.
    if args.channels == 0 and args.file:
        try:
            wf = wave.open(args.file, "rb")
            args.channels = wf.getnchannels()
            wf.close()
            print(f"[ssl_demo] auto-detected -c {args.channels} from WAV "
                  f"header")
        except Exception as exc:
            print(f"Cannot probe {args.file}: {exc}", file=sys.stderr)
            sys.exit(1)
    if args.channels == 0:
        args.channels = 2  # default for synthetic / live without -c

    if args.channels < 2:
        print(f"Error: -c must be >= 2 (got {args.channels})",
              file=sys.stderr)
        sys.exit(1)
    if args.channels >= 3 and args.flip:
        print(
            "Error: --flip is 2-ch only. For 3+ ch use --azimuth-offset 180.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Resolve mic_distance with per-mode default + warning.
    user_set_d = args.mic_distance is not None
    if args.channels == 2:
        args.mic_distance = (args.mic_distance if user_set_d
                             else DEFAULT_MIC_DISTANCE_2CH)
        warn_default_mic_distance(user_set_d, args.mic_distance,
                                  "2-ch (stereo) demo")
    else:
        args.mic_distance = (args.mic_distance if user_set_d
                             else DEFAULT_MIC_DISTANCE_3CH)
        warn_default_mic_distance(user_set_d, args.mic_distance,
                                  "3-ch equilateral demo")

    # Dispatch.
    if args.channels == 2:
        if args.test:
            sys.exit(run_test_2ch(args.mic_distance, args.rate, args.flip))
        if args.file:
            sys.exit(run_file_2ch(args.file, args.mic_distance,
                                  args.verbose, args.flip))
        if args.live:
            sys.exit(run_live_2ch(args.mic_distance, args.rate,
                                  args.input_device, args.verbose,
                                  args.flip))
    else:
        if args.test:
            sys.exit(run_test_multi(args))
        if args.file:
            sys.exit(run_file_multi(args.file, args))
        if args.live:
            sys.exit(run_live_multi(args))


if __name__ == "__main__":
    main()
