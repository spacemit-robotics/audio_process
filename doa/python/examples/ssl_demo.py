#!/usr/bin/env python3
"""
Sound Source Localization demo (Python)

Usage:
    ssl_demo.py -t [-d mic_dist] [-r rate] [--flip]         Synthetic accuracy test
    ssl_demo.py -f <stereo.wav> [-d mic_dist] [-v] [--flip] Process WAV file
    ssl_demo.py -l [-d mic_dist] [-r rate] [-i dev] [--flip] Live capture
"""

import argparse
import math
import sys
import time
import wave

try:
    import numpy as np
except ImportError:
    print("Error: numpy is required.  pip install numpy")
    sys.exit(1)

try:
    from spacemit_audio_process import SoundLocator, SoundLocatorConfig
except ImportError as e:
    print("Error: cannot import spacemit_audio_process")
    print("Install the package or set PYTHONPATH to the build directory:")
    print("  export PYTHONPATH=/path/to/doa/build/spacemit_audio_process")
    print(f"\nDetails: {e}")
    sys.exit(1)


def report_angle(doa: float, flip_angle: bool) -> float:
    angle = 180.0 - doa if flip_angle else doa
    return max(0.0, min(180.0, angle))


def print_angle_mapping(flip_angle: bool) -> None:
    if flip_angle:
        print("Angle mapping: flipped (reported = 180 - raw DOA)")


# -------------------------------------------------------------------
# Synthetic accuracy test
# -------------------------------------------------------------------
def run_test(mic_dist: float, sample_rate: int, flip_angle: bool) -> int:
    print("=== Synthetic Accuracy Test ===")
    print(f"Mic distance: {mic_dist:.4f} m | Sample rate: {sample_rate} Hz\n")
    print_angle_mapping(flip_angle)
    if flip_angle:
        print()

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

    test_angles = [
        90, 85, 80, 75, 70, 60, 45, 30, 15, 0,
        95, 100, 120, 135, 150, 180,
    ]

    passed = 0
    failed = 0

    rng = np.random.default_rng(42)

    for angle in test_angles:
        loc.reset()

        phys_angle = 90.0 - angle
        theta_rad = math.radians(phys_angle)
        delay_sec = mic_dist * math.sin(theta_rad) / c
        delay_samples = -delay_sec * fs

        # Broadband white noise for ch0 (with padding for sinc kernel)
        ch0_padded = rng.standard_normal(total_samples + 2 * sinc_half).astype(
            np.float32
        ) * 0.5

        ch0 = ch0_padded[sinc_half : sinc_half + total_samples].copy()

        # ch1 = delayed version of ch0 via windowed-sinc interpolation
        ch1 = np.zeros(total_samples, dtype=np.float32)
        for i in range(total_samples):
            src_pos = float(i + sinc_half) - delay_samples
            src_int = int(math.floor(src_pos))
            val = 0.0
            for k in range(src_int - sinc_half + 1, src_int + sinc_half + 1):
                if 0 <= k < len(ch0_padded):
                    x = src_pos - float(k)
                    sinc = 1.0 if abs(x) < 1e-6 else math.sin(math.pi * x) / (math.pi * x)
                    w = 0.5 * (1.0 + math.cos(math.pi * x / sinc_half))
                    val += float(ch0_padded[k]) * sinc * w
            ch1[i] = val

        # Feed frames
        ready = False
        for offset in range(0, total_samples, cfg.frame_size):
            end = offset + cfg.frame_size
            if loc.process_separate(ch0[offset:end], ch1[offset:end]):
                ready = True

        if not ready:
            print(f"  {report_angle(angle, flip_angle):6.1f}\u00b0 \u2192 NO RESULT")
            failed += 1
            continue

        expected = report_angle(angle, flip_angle)
        est = report_angle(loc.doa, flip_angle)
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
            f"  {expected:6.1f}\u00b0 \u2192 {est:7.2f}\u00b0 | err: {err_abs:.2f}\u00b0 "
            f"({err_pct:.1f}%) | conf: {loc.confidence:.3f} | "
            f"delay: {delay_samples:.3f} samp | {status}"
        )

    print(f"\n--- Results: {passed}/{passed + failed} passed ---")
    return 1 if failed > 0 else 0


# -------------------------------------------------------------------
# WAV file mode
# -------------------------------------------------------------------
def run_file(path: str, mic_dist: float, verbose: bool, flip_angle: bool) -> int:
    try:
        wf = wave.open(path, "rb")
    except Exception as e:
        print(f"Cannot open {path}: {e}", file=sys.stderr)
        return 1

    channels = wf.getnchannels()
    sample_rate = wf.getframerate()
    n_frames = wf.getnframes()
    sampwidth = wf.getsampwidth()

    if channels != 2:
        print(f"WAV must be stereo (got {channels} channels)", file=sys.stderr)
        wf.close()
        return 1

    if sampwidth != 2:
        print(f"Only PCM16 WAV is supported (got {sampwidth * 8}-bit)", file=sys.stderr)
        wf.close()
        return 1

    raw = wf.readframes(n_frames)
    wf.close()
    samples = np.frombuffer(raw, dtype=np.int16)

    duration = n_frames / sample_rate
    print(f"=== WAV File: {path} ===")
    print(f"Sample rate: {sample_rate} Hz | Samples: {n_frames} | Duration: {duration:.2f} s")
    print_angle_mapping(flip_angle)

    cfg = SoundLocatorConfig()
    cfg.sample_rate = sample_rate
    cfg.mic_distance = mic_dist
    cfg.frame_size = 512
    cfg.avg_frames = 4
    print(f"Configured max TDOA: \u00b1{mic_dist / cfg.sound_speed:.6f}s (mic_distance={mic_dist:.4f}m)")

    loc = SoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    offset = 0
    frame_no = 0
    t_start = time.monotonic()

    while offset < n_frames:
        chunk = min(cfg.frame_size, n_frames - offset)
        interleaved = samples[offset * 2 : (offset + chunk) * 2]
        if loc.process_int16(interleaved):
            frame_no += 1
            if verbose:
                if loc.is_valid:
                    max_tdoa = cfg.mic_distance / cfg.sound_speed
                    delay_ratio = abs(loc.tdoa) / max_tdoa
                    print(
                        f"[Frame {frame_no:03d}] DOA: {report_angle(loc.doa, flip_angle):7.2f}\u00b0 | "
                        f"TDOA: {loc.tdoa:+.6f}s | Ratio: {delay_ratio:.3f} | "
                        f"Conf: {loc.confidence:.3f}"
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
        print(f"\nest angle = {report_angle(loc.average_doa, flip_angle):.1f}")
    else:
        print("\nest angle = N/A (no valid frames)")
    print(f"Process: {process_ms:.1f} ms | Audio: {duration:.2f} s | RTF: {rtf:.4f}")
    return 0


# -------------------------------------------------------------------
# Live capture mode (requires spacemit_audio)
# -------------------------------------------------------------------
def run_live(mic_dist: float, sample_rate: int, device_index: int,
             verbose: bool, flip_angle: bool) -> int:
    try:
        import spacemit_audio
    except ImportError:
        print("Error: spacemit_audio is required for live capture.")
        print("Install spacemit-audio in the current Python environment.")
        return 1

    print("=== Live Capture ===")

    cfg = SoundLocatorConfig()
    cfg.sample_rate = sample_rate
    cfg.mic_distance = mic_dist
    cfg.frame_size = 512
    cfg.avg_frames = 4
    print(
        f"Mic distance: {mic_dist:.4f} m | Sample rate: {sample_rate} Hz | "
        f"Device: {device_index}"
    )
    print(f"Configured max TDOA: \u00b1{mic_dist / cfg.sound_speed:.6f}s")
    print_angle_mapping(flip_angle)
    print("Press Ctrl+C to stop.")
    print("If no DOA is printed, speak or clap near one side of the microphone array.\n")

    loc = SoundLocator(cfg)
    if not loc.initialize():
        print("Initialize failed", file=sys.stderr)
        return 1

    chunk_size = cfg.frame_size * 2 * 2  # stereo, int16
    spacemit_audio.init(
        sample_rate=sample_rate,
        channels=2,
        chunk_size=chunk_size,
        capture_device=device_index,
        player_device=-1,
    )
    capture = spacemit_audio.AudioCapture(device_index)

    state = {
        "frame_no": 0,
        "callbacks": 0,
        "frames": 0,
        "valid": 0,
        "confidence": 0.0,
    }

    def callback(data: bytes):
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
                max_tdoa = cfg.mic_distance / cfg.sound_speed
                delay_ratio = abs(loc.tdoa) / max_tdoa
                print(
                    f"\r[{state['frame_no']:04d}] DOA: "
                    f"{report_angle(loc.doa, flip_angle):7.2f}\u00b0 | "
                    f"TDOA: {loc.tdoa:+.6f}s | Ratio: {delay_ratio:.3f} | "
                    f"Conf: {loc.confidence:.3f}    ",
                    end="", flush=True,
                )
            elif verbose:
                print(
                    f"\r[{state['frame_no']:04d}] DOA: ------- | "
                    f"Conf: {loc.confidence:.3f} (below threshold)    ",
                    end="", flush=True,
                )

    capture.set_callback(callback)
    if not capture.start(sample_rate=sample_rate, channels=2, chunk_size=chunk_size):
        print("Failed to start capture", file=sys.stderr)
        capture.close()
        return 1

    try:
        last_status = time.monotonic()
        while True:
            time.sleep(0.1)
            now = time.monotonic()
            if state["valid"] == 0 and now - last_status >= 1.0:
                last_status = now
                print(
                    "\rWaiting for valid DOA... "
                    f"callbacks: {state['callbacks']}, frames: {state['frames']}, "
                    f"conf: {state['confidence']:.3f}    ",
                    end="", flush=True,
                )
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


# -------------------------------------------------------------------
# Main
# -------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Sound Source Localization demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Modes:\n"
            "  -t    Synthetic accuracy test\n"
            "  -f    Process a stereo WAV file\n"
            "  -l    Live capture from microphone (needs spacemit_audio)"
        ),
    )

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("-t", "--test", action="store_true",
                      help="Synthetic accuracy test")
    mode.add_argument("-f", "--file", metavar="WAV",
                      help="Process a stereo WAV file")
    mode.add_argument("-l", "--live", action="store_true",
                      help="Live capture from microphone")

    parser.add_argument("-d", "--mic-dist", type=float, default=0.058,
                        help="Microphone distance in meters (default: 0.058)")
    parser.add_argument("-r", "--rate", type=int, default=16000,
                        help="Sample rate in Hz (default: 16000)")
    parser.add_argument("-i", "--input-device", type=int, default=-1,
                        help="Audio capture device index (default: -1 = auto)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Print per-frame details (file/live mode)")
    parser.add_argument("--flip", action="store_true",
                        help="Report mirrored angle: 0->180, 50->130, 180->0")

    args = parser.parse_args()

    if args.test:
        sys.exit(run_test(args.mic_dist, args.rate, args.flip))
    elif args.file:
        sys.exit(run_file(args.file, args.mic_dist, args.verbose, args.flip))
    elif args.live:
        sys.exit(run_live(args.mic_dist, args.rate, args.input_device,
                          args.verbose, args.flip))


if __name__ == "__main__":
    main()
