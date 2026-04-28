/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ssl_demo — Sound Source Localization demo
 *
 * Usage:
 *   ssl_demo -t [-d mic_dist] [-r rate] [--flip]          Synthetic accuracy test
 *   ssl_demo -f <stereo.wav> [-d mic_dist] [-v] [--flip]  Process WAV file
 *   ssl_demo -l [-d mic_dist] [-r rate] [-i dev] [--flip] Live capture
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "doa_service.h"

#ifdef HAS_SPACEMIT_AUDIO
#include "audio_base.hpp"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float ReportAngle(float doa, bool flip_angle) {
    float angle = flip_angle ? 180.0f - doa : doa;
    return std::max(0.0f, std::min(180.0f, angle));
}

static void PrintAngleMapping(bool flip_angle) {
    if (flip_angle) {
        printf("Angle mapping: flipped (reported = 180 - raw DOA)\n");
    }
}

// -----------------------------------------------------------------------
// Minimal WAV reader (PCM16 only)
// -----------------------------------------------------------------------
struct WavHeader {
    char riff[4];
    uint32_t file_size;
    char wave[4];
    char fmt_id[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
};

static bool ReadWav(const char* path, std::vector<int16_t>& samples,
                    int& channels, int& sample_rate) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "Cannot open %s\n", path);
        return false;
    }

    WavHeader hdr;
    if (fread(&hdr, sizeof(hdr), 1, f) != 1) {
        fclose(f);
        return false;
    }

    if (memcmp(hdr.riff, "RIFF", 4) || memcmp(hdr.wave, "WAVE", 4)) {
        fprintf(stderr, "Not a WAV file\n"); fclose(f); return false;
    }
    if (hdr.audio_format != 1 || hdr.bits_per_sample != 16) {
        fprintf(stderr, "Only PCM16 WAV is supported\n"); fclose(f); return false;
    }

    channels = hdr.channels;
    sample_rate = static_cast<int>(hdr.sample_rate);

    // Skip to "data" chunk
    char chunk_id[4];
    uint32_t chunk_size;
    while (fread(chunk_id, 4, 1, f) == 1 && fread(&chunk_size, 4, 1, f) == 1) {
        if (memcmp(chunk_id, "data", 4) == 0) {
            size_t n = chunk_size / sizeof(int16_t);
            samples.resize(n);
            size_t read_count = fread(samples.data(), sizeof(int16_t), n, f);
            fclose(f);
            if (read_count != n) {
                fprintf(stderr, "Failed to read WAV data\n");
                return false;
            }
            return true;
        }
        fseek(f, chunk_size, SEEK_CUR);
    }

    fclose(f);
    fprintf(stderr, "No data chunk found\n");
    return false;
}

// -----------------------------------------------------------------------
// Synthetic test: generate delayed stereo signal, verify DOA accuracy
// -----------------------------------------------------------------------
static int RunTest(float mic_dist, int sample_rate, bool flip_angle) {
    printf("=== Synthetic Accuracy Test ===\n");
    printf("Mic distance: %.4f m | Sample rate: %d Hz\n\n", mic_dist, sample_rate);
    PrintAngleMapping(flip_angle);
    if (flip_angle) {
        printf("\n");
    }

    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.mic_distance = mic_dist;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    cfg.confidence_threshold = 0.05f;

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    printf("Max delay: %d samples\n\n", loc.GetMaxDelaySamples());

    const float c = cfg.sound_speed;
    const float fs = static_cast<float>(sample_rate);
    const int total_samples = cfg.frame_size * cfg.avg_frames;

    // Windowed-sinc interpolation kernel half-width
    constexpr int kSincHalf = 16;

    // Test angles in [0, 180] coordinate system
    // 90° = broadside, 0° = endfire (ch1 side), 180° = endfire (ch0 side)
    const float test_angles[] = {
        90.0f, 85.0f, 80.0f, 75.0f, 70.0f, 60.0f, 45.0f, 30.0f, 15.0f, 0.0f,
        95.0f, 100.0f, 120.0f, 135.0f, 150.0f, 180.0f
    };

    int pass = 0, fail = 0;

    for (float angle : test_angles) {
        loc.Reset();

        // Convert from [0,180] coordinate to physical angle for delay calculation
        // Physical angle = 90° - DOA (where 90° is broadside)
        float phys_angle = 90.0f - angle;
        float theta_rad = phys_angle * static_cast<float>(M_PI) / 180.0f;
        float delay_sec = mic_dist * std::sin(theta_rad) / c;
        float delay_samples = -delay_sec * fs;

        // Generate broadband white noise for ch0
        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.0f, 0.5f);
        std::vector<float> ch0(total_samples + 2 * kSincHalf);
        for (auto& s : ch0) s = noise(rng);

        // Create ch1 as delayed version of ch0 using windowed-sinc interpolation
        std::vector<float> ch1(total_samples);
        std::vector<float> ch0_trimmed(total_samples);
        for (int i = 0; i < total_samples; ++i) {
            ch0_trimmed[i] = ch0[i + kSincHalf];

            float src_pos = static_cast<float>(i + kSincHalf) - delay_samples;
            float val = 0.0f;
            int src_int = static_cast<int>(std::floor(src_pos));
            for (int k = src_int - kSincHalf + 1; k <= src_int + kSincHalf; ++k) {
                if (k >= 0 && k < static_cast<int>(ch0.size())) {
                    float x = src_pos - static_cast<float>(k);
                    // Sinc function
                    float sinc = (std::fabs(x) < 1e-6f)
                        ? 1.0f
                        : std::sin(static_cast<float>(M_PI) * x)
                            / (static_cast<float>(M_PI) * x);
                    // Hann window on sinc kernel
                    float w = 0.5f * (1.0f + std::cos(
                        static_cast<float>(M_PI) * x / kSincHalf));
                    val += ch0[k] * sinc * w;
                }
            }
            ch1[i] = val;
        }

        // Feed frames
        bool ready = false;
        for (int offset = 0; offset < total_samples; offset += cfg.frame_size) {
            if (loc.Process(ch0_trimmed.data() + offset, ch1.data() + offset,
                            cfg.frame_size)) {
                ready = true;
            }
        }

        if (!ready) {
            printf("  %6.1f° → NO RESULT\n", ReportAngle(angle, flip_angle));
            ++fail;
            continue;
        }

        float expected = ReportAngle(angle, flip_angle);
        float est = ReportAngle(loc.GetDOA(), flip_angle);
        float err_abs = std::fabs(est - expected);
        // Avoid division by zero for 0° and 180° endfire
        float err_pct = err_abs;
        if (expected > 1.0f && expected < 179.0f) {
            err_pct = err_abs / std::min(expected, 180.0f - expected) * 100.0f;
        }

        constexpr float kMaxErrPct = 3.0f;
        constexpr float kMaxErrDeg = 2.5f;
        const bool passed_threshold = (err_pct < kMaxErrPct || err_abs < kMaxErrDeg);
        const char* status = passed_threshold ? "PASS" : "FAIL";
        if (status[0] == 'P') {
            ++pass;
        } else {
            ++fail;
        }

        printf("  %6.1f° → %7.2f° | err: %.2f° (%.1f%%) | conf: %.3f | delay: %.3f samp | %s\n",
                expected, est, err_abs, err_pct, loc.GetConfidence(),
                delay_samples, status);
    }

    printf("\n--- Results: %d/%d passed ---\n", pass, pass + fail);
    return fail > 0 ? 1 : 0;
}

// -----------------------------------------------------------------------
// WAV file mode
// -----------------------------------------------------------------------
static int RunFile(const char* path, float mic_dist, bool verbose, bool flip_angle) {
    std::vector<int16_t> samples;
    int channels, sample_rate;
    if (!ReadWav(path, samples, channels, sample_rate)) return 1;

    if (channels != 2) {
        fprintf(stderr, "WAV must be stereo (got %d channels)\n", channels);
        return 1;
    }

    printf("=== WAV File: %s ===\n", path);
    printf("Sample rate: %d Hz | Samples: %zu | Duration: %.2f s\n",
            sample_rate, samples.size() / 2,
            static_cast<float>(samples.size() / 2) / sample_rate);
    PrintAngleMapping(flip_angle);

    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.mic_distance = mic_dist;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    printf("Configured max TDOA: ±%.6fs (mic_distance=%.4fm)\n",
            mic_dist / cfg.sound_speed, mic_dist);

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const size_t num_frames_total = samples.size() / 2;
    const float audio_duration =
        static_cast<float>(num_frames_total) / sample_rate;
    size_t offset = 0;
    int frame_no = 0;

    auto t_start = std::chrono::steady_clock::now();

    while (offset < num_frames_total) {
        size_t chunk = std::min(static_cast<size_t>(cfg.frame_size),
                num_frames_total - offset);
        if (loc.Process(samples.data() + offset * 2, chunk)) {
            ++frame_no;
            if (verbose) {
                if (loc.IsValid()) {
                    const float max_tdoa = cfg.mic_distance / cfg.sound_speed;
                    const float delay_ratio = std::fabs(loc.GetTDOA()) / max_tdoa;
                    printf("[Frame %03d] DOA: %7.2f° | TDOA: %+.6fs | Ratio: %.3f | Conf: %.3f\n",
                            frame_no, ReportAngle(loc.GetDOA(), flip_angle),
                            loc.GetTDOA(), delay_ratio,
                            loc.GetConfidence());
                } else {
                    printf("[Frame %03d] DOA: ------- | Conf: %.3f (below threshold)\n",
                            frame_no, loc.GetConfidence());
                }
            }
        }
        offset += chunk;
    }

    auto t_end = std::chrono::steady_clock::now();
    float process_ms = std::chrono::duration<float, std::milli>(
        t_end - t_start).count();
    float rtf = (process_ms / 1000.0f) / audio_duration;

    if (loc.GetResultCount() > 0) {
        printf("\nest angle = %.1f\n", ReportAngle(loc.GetAverageDOA(), flip_angle));
    } else {
        printf("\nest angle = N/A (no valid frames)\n");
    }
    printf("Process: %.1f ms | Audio: %.2f s | RTF: %.4f\n",
            process_ms, audio_duration, rtf);

    return 0;
}

// -----------------------------------------------------------------------
// Live capture mode (requires spacemit_audio)
// -----------------------------------------------------------------------
#ifdef HAS_SPACEMIT_AUDIO
static volatile std::sig_atomic_t g_stop_requested = 0;

static void HandleSignal(int) {
    g_stop_requested = 1;
}

static int RunLive(
    float mic_dist,
    int sample_rate,
    int device_index,
    bool verbose,
    bool flip_angle) {
    printf("=== Live Capture ===\n");
    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.mic_distance = mic_dist;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    printf("Mic distance: %.4f m | Sample rate: %d Hz | Device: %d\n",
            mic_dist, sample_rate, device_index);
    printf("Configured max TDOA: ±%.6fs\n", mic_dist / cfg.sound_speed);
    PrintAngleMapping(flip_angle);
    printf("Press Ctrl+C to stop.\n");
    printf("If no DOA is printed, speak or clap near one side of the microphone array.\n\n");

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    int chunk_size = cfg.frame_size * 2 * 2;  // stereo, int16
    SpacemitAudio::Init(sample_rate, 2, chunk_size, device_index, -1);
    SpacemitAudio::AudioCapture capture(device_index);

    int frame_no = 0;
    std::atomic<int> callback_count{0};
    std::atomic<int> ready_count{0};
    std::atomic<int> valid_count{0};
    std::atomic<int> last_confidence_milli{0};

    capture.SetCallback([&](const uint8_t* data, size_t size) {
        ++callback_count;
        size_t num_frames = size / (2 * sizeof(int16_t));
        const auto* pcm = reinterpret_cast<const int16_t*>(data);

        if (loc.Process(pcm, num_frames)) {
            ++ready_count;
            last_confidence_milli.store(
                static_cast<int>(loc.GetConfidence() * 1000.0f));
            ++frame_no;
            if (loc.IsValid()) {
                ++valid_count;
                const float max_tdoa = cfg.mic_distance / cfg.sound_speed;
                const float delay_ratio = std::fabs(loc.GetTDOA()) / max_tdoa;
                printf("\r[%04d] DOA: %7.2f° | TDOA: %+.6fs | Ratio: %.3f | Conf: %.3f    ",
                        frame_no, ReportAngle(loc.GetDOA(), flip_angle),
                        loc.GetTDOA(), delay_ratio,
                        loc.GetConfidence());
                fflush(stdout);
            } else if (verbose) {
                printf("\r[%04d] DOA: ------- | Conf: %.3f (below threshold)    ",
                        frame_no, loc.GetConfidence());
                fflush(stdout);
            }
        }
    });

    if (!capture.Start(sample_rate, 2, chunk_size)) {
        fprintf(stderr, "Failed to start capture\n");
        return 1;
    }

    g_stop_requested = 0;
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    auto last_status = std::chrono::steady_clock::now();
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto now = std::chrono::steady_clock::now();
        if (valid_count.load() == 0 &&
                now - last_status >= std::chrono::seconds(1)) {
            last_status = now;
            printf("\rWaiting for valid DOA... callbacks: %d, frames: %d, conf: %.3f    ",
                    callback_count.load(),
                    ready_count.load(),
                    last_confidence_milli.load() / 1000.0f);
            fflush(stdout);
        }
    }

    capture.Stop();
    printf("\nStopped. callbacks: %d, frames: %d, valid: %d\n",
            callback_count.load(), ready_count.load(), valid_count.load());
    return 0;
}
#endif  // HAS_SPACEMIT_AUDIO

// -----------------------------------------------------------------------
// Usage
// -----------------------------------------------------------------------
static void PrintUsage(const char* prog) {
    printf("Usage:\n");
    printf("  %s -t [-d mic_dist] [-r rate] [--flip]\n", prog);
    printf("  %s -f <stereo.wav> [-d mic_dist] [-v] [--flip]\n", prog);
#ifdef HAS_SPACEMIT_AUDIO
    printf("  %s -l [-d mic_dist] [-r rate] [-i device] [--flip]\n", prog);
#endif
    printf("\nMode:\n");
    printf("  -t, --test      Synthetic accuracy test\n");
    printf("  -f, --file      Process a stereo WAV file\n");
#ifdef HAS_SPACEMIT_AUDIO
    printf("  -l, --live      Live capture from microphone\n");
#endif
    printf("\nOptions:\n");
    printf("  -d  Microphone distance in meters (default: 0.058)\n");
    printf("  -r  Sample rate in Hz (default: 16000)\n");
    printf("  -i  Audio capture device index (default: -1 = auto)\n");
    printf("  -v, --verbose   Print per-frame details (file/live mode)\n");
    printf("  --flip          Report mirrored angle: 0->180, 50->130, 180->0\n");
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    float mic_dist = 0.058f;
    int sample_rate = 16000;
    int device_index = -1;
    bool verbose = false;
    bool flip_angle = false;
    const char* mode = nullptr;
    const char* wav_path = nullptr;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
            mode = "test";
        } else if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0) {
            mode = "file";
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                wav_path = argv[++i];
            }
        } else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--live") == 0) {
            mode = "live";
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--flip") == 0) {
            flip_angle = true;
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            mic_dist = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            sample_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            device_index = atoi(argv[++i]);
        }
    }

    if (!mode) {
        PrintUsage(argv[0]);
        return 1;
    }

    if (strcmp(mode, "test") == 0) {
        return RunTest(mic_dist, sample_rate, flip_angle);
    }

    if (strcmp(mode, "file") == 0) {
        if (!wav_path) {
            fprintf(stderr, "Error: --file requires a WAV path\n\n");
            PrintUsage(argv[0]);
            return 1;
        }
        return RunFile(wav_path, mic_dist, verbose, flip_angle);
    }

#ifdef HAS_SPACEMIT_AUDIO
    if (strcmp(mode, "live") == 0) {
        return RunLive(mic_dist, sample_rate, device_index, verbose, flip_angle);
    }
#endif

    PrintUsage(argv[0]);
    return 1;
}
