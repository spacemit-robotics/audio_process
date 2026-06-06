/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ssl_demo — unified Sound Source Localization demo (2-ch and 3-ch+)
 *
 * Usage:
 *   ssl_demo -c 2 -t [-d M] [-r RATE] [--flip]
 *   ssl_demo -c 2 -f stereo.wav [-d M] [-v] [--flip]
 *   ssl_demo -c 2 -l [-d M] [-r RATE] [-i DEV] [--flip]
 *
 *   ssl_demo -c 3 -t [-d M] [--angle DEG] [--sweep A:B:S] [--noise-snr DB]
 *                    [--azimuth-offset DEG] [--positions SPEC]
 *                    [--avg-seconds S]
 *                    [--quality-threshold F] [--margin-threshold F]
 *                    [--closure-threshold-samples F]
 *                    [--closure-threshold-fraction F] [--max-frequency-hz F]
 *   ssl_demo -c 3 -f 3ch.wav [-d M] [-v] [--positions SPEC]
 *                    [--azimuth-offset DEG] [--avg-seconds S]
 *   ssl_demo -c 3 -l [-d M] [-r RATE] [-i DEV] [-v]
 *                    [--positions SPEC] [--azimuth-offset DEG]
 *                    [--avg-seconds S]
 *
 *   ssl_demo -h | --help
 *
 * Channel count is selected with -c N (N=2 dispatches to SoundLocator;
 * N>=3 dispatches to MultiSoundLocator). For -f mode without -c, the WAV
 * channel count is auto-detected.
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
#include <exception>
#include <limits>
#include <mutex>
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

// ===========================================================================
// Shared helpers
// ===========================================================================

namespace {

constexpr float kDefault2chMicDistance = 0.058f;
constexpr float kDefault3chMicDistance = 0.063f;
constexpr float kDefaultLiveAvgSeconds = 10.0f;
constexpr float kDefaultLiveMinSignalRms = 0.003f;
constexpr int kSyntheticSourceSeed = 2;
constexpr int kSyntheticToneCount = 29;
constexpr double kSyntheticMinHz = 300.0;
constexpr double kSyntheticMaxHz = 3800.0;

struct WavData {
    std::vector<int16_t> samples;
    int channels = 0;
    int sample_rate = 0;
};

struct WavHeaderInfo {
    int channels = 0;
    int sample_rate = 0;
};

float NormalizeAngle360(float degrees) {
    float v = std::fmod(degrees, 360.0f);
    if (v < 0.0f) v += 360.0f;
    if (v >= 360.0f) v -= 360.0f;
    return v;
}

float CircularError(float a, float b) {
    float diff = std::fabs(NormalizeAngle360(a) - NormalizeAngle360(b));
    return std::min(diff, 360.0f - diff);
}

bool ReadWavHeader(const char* path, WavHeaderInfo& info) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::fprintf(stderr, "Cannot open %s\n", path);
        return false;
    }

    char riff[4]; uint32_t riff_size = 0; char wave[4];
    if (std::fread(riff, 1, 4, f) != 4
        || std::fread(&riff_size, sizeof(riff_size), 1, f) != 1
        || std::fread(wave, 1, 4, f) != 4
        || std::memcmp(riff, "RIFF", 4) != 0
        || std::memcmp(wave, "WAVE", 4) != 0) {
        std::fclose(f);
        std::fprintf(stderr, "Not a WAV file\n");
        return false;
    }

    while (!std::feof(f)) {
        char chunk_id[4]; uint32_t chunk_size = 0;
        if (std::fread(chunk_id, 1, 4, f) != 4
            || std::fread(&chunk_size, sizeof(chunk_size), 1, f) != 1) break;

        if (std::memcmp(chunk_id, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                std::fclose(f);
                std::fprintf(stderr, "Invalid WAV fmt chunk\n");
                return false;
            }
            uint16_t audio_format = 0, channels = 0, bits_per_sample = 0;
            uint32_t sample_rate = 0, byte_rate = 0; uint16_t block_align = 0;
            if (std::fread(&audio_format, sizeof(audio_format), 1, f) != 1
                || std::fread(&channels, sizeof(channels), 1, f) != 1
                || std::fread(&sample_rate, sizeof(sample_rate), 1, f) != 1
                || std::fread(&byte_rate, sizeof(byte_rate), 1, f) != 1
                || std::fread(&block_align, sizeof(block_align), 1, f) != 1
                || std::fread(&bits_per_sample, sizeof(bits_per_sample), 1, f) != 1) {
                std::fclose(f);
                std::fprintf(stderr, "Failed to read WAV fmt chunk\n");
                return false;
            }
            std::fclose(f);
            if (audio_format != 1 || bits_per_sample != 16) {
                std::fprintf(stderr, "Only PCM16 WAV is supported\n");
                return false;
            }
            if (channels == 0) {
                std::fprintf(stderr, "Invalid WAV channel count: 0\n");
                return false;
            }
            info.channels = static_cast<int>(channels);
            info.sample_rate = static_cast<int>(sample_rate);
            return true;
        }

        std::fseek(f, static_cast<int64_t>(chunk_size), SEEK_CUR);
        if (chunk_size % 2 != 0) std::fseek(f, 1, SEEK_CUR);
    }

    std::fclose(f);
    std::fprintf(stderr, "No fmt chunk found\n");
    return false;
}

// Read a PCM16 WAV of any channel count.
bool ReadWav(const char* path, WavData& wav) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::fprintf(stderr, "Cannot open %s\n", path);
        return false;
    }

    char riff[4]; uint32_t riff_size = 0; char wave[4];
    if (std::fread(riff, 1, 4, f) != 4
        || std::fread(&riff_size, sizeof(riff_size), 1, f) != 1
        || std::fread(wave, 1, 4, f) != 4
        || std::memcmp(riff, "RIFF", 4) != 0
        || std::memcmp(wave, "WAVE", 4) != 0) {
        std::fclose(f);
        std::fprintf(stderr, "Not a WAV file\n");
        return false;
    }

    bool got_fmt = false;
    uint16_t audio_format = 0, channels = 0, bits_per_sample = 0;
    uint32_t sample_rate = 0;

    while (!std::feof(f)) {
        char chunk_id[4]; uint32_t chunk_size = 0;
        if (std::fread(chunk_id, 1, 4, f) != 4
            || std::fread(&chunk_size, sizeof(chunk_size), 1, f) != 1) break;

        if (std::memcmp(chunk_id, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                std::fclose(f);
                return false;
            }
            uint32_t byte_rate = 0; uint16_t block_align = 0;
            if (std::fread(&audio_format, sizeof(audio_format), 1, f) != 1
                || std::fread(&channels, sizeof(channels), 1, f) != 1
                || std::fread(&sample_rate, sizeof(sample_rate), 1, f) != 1
                || std::fread(&byte_rate, sizeof(byte_rate), 1, f) != 1
                || std::fread(&block_align, sizeof(block_align), 1, f) != 1
                || std::fread(&bits_per_sample, sizeof(bits_per_sample), 1, f) != 1) {
                std::fclose(f);
                std::fprintf(stderr, "Failed to read WAV fmt chunk\n");
                return false;
            }
            if (chunk_size > 16) {
                std::fseek(f, static_cast<int64_t>(chunk_size - 16), SEEK_CUR);
            }
            got_fmt = true;
        } else if (std::memcmp(chunk_id, "data", 4) == 0) {
            if (!got_fmt) {
                std::fclose(f);
                std::fprintf(stderr, "WAV data chunk appeared before fmt\n");
                return false;
            }
            if (audio_format != 1 || bits_per_sample != 16) {
                std::fclose(f);
                std::fprintf(stderr, "Only PCM16 WAV is supported\n");
                return false;
            }
            wav.channels = static_cast<int>(channels);
            wav.sample_rate = static_cast<int>(sample_rate);
            wav.samples.resize(chunk_size / sizeof(int16_t));
            const size_t got = std::fread(wav.samples.data(), sizeof(int16_t),
                wav.samples.size(), f);
            std::fclose(f);
            if (got != wav.samples.size()) {
                std::fprintf(stderr, "Failed to read WAV data\n");
                return false;
            }
            return true;
        } else {
            std::fseek(f, static_cast<int64_t>(chunk_size), SEEK_CUR);
        }
        if (chunk_size % 2 != 0) std::fseek(f, 1, SEEK_CUR);
    }

    std::fclose(f);
    std::fprintf(stderr, "No data chunk found\n");
    return false;
}

void WarnIfDefaultMicDistance(bool user_set, float used, const char* mode_tag) {
    if (!user_set) {
        std::fprintf(stderr,
            "[ssl_demo] WARNING: -d / --mic-distance not set; using demo default "
            "%.4f m for %s. Production code MUST measure and set the real "
            "physical mic spacing — endfire angles fail silently otherwise.\n",
            used, mode_tag);
    }
}

#ifdef HAS_SPACEMIT_AUDIO
volatile std::sig_atomic_t g_stop_requested = 0;
void HandleSignal(int) { g_stop_requested = 1; }
#endif

// ===========================================================================
// 2-channel mode (SoundLocator)
// ===========================================================================

float ReportAngle2ch(float doa, bool flip) {
    float a = flip ? 180.0f - doa : doa;
    return std::max(0.0f, std::min(180.0f, a));
}

int RunTest_2ch(float mic_distance, int sample_rate, bool flip) {
    std::printf("=== 2-ch Synthetic Accuracy Test ===\n");
    std::printf("Mic distance: %.4f m | Sample rate: %d Hz\n\n",
                mic_distance, sample_rate);
    if (flip) std::printf("Angle mapping: flipped (reported = 180 - raw DOA)\n\n");

    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.mic_distance = mic_distance;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    cfg.confidence_threshold = 0.05f;

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }
    std::printf("Max delay: %d samples\n\n", loc.GetMaxDelaySamples());

    const float c = cfg.sound_speed;
    const float fs = static_cast<float>(sample_rate);
    const int total_samples = cfg.frame_size * cfg.avg_frames;
    constexpr int kSincHalf = 16;

    const float test_angles[] = {
        90.0f, 85.0f, 80.0f, 75.0f, 70.0f, 60.0f, 45.0f, 30.0f, 15.0f, 0.0f,
        95.0f, 100.0f, 120.0f, 135.0f, 150.0f, 180.0f
    };

    int pass = 0, fail = 0;
    for (float angle : test_angles) {
        loc.Reset();
        const float phys = 90.0f - angle;
        const float theta = phys * static_cast<float>(M_PI) / 180.0f;
        const float delay_sec = mic_distance * std::sin(theta) / c;
        const float delay_samples = -delay_sec * fs;

        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.0f, 0.5f);
        std::vector<float> ch0(total_samples + 2 * kSincHalf);
        for (auto& s : ch0) s = noise(rng);

        std::vector<float> ch1(total_samples), ch0_trim(total_samples);
        for (int i = 0; i < total_samples; ++i) {
            ch0_trim[i] = ch0[i + kSincHalf];
            const float src = static_cast<float>(i + kSincHalf) - delay_samples;
            const int center = static_cast<int>(std::floor(src));
            float v = 0.0f;
            for (int k = center - kSincHalf + 1; k <= center + kSincHalf; ++k) {
                if (k < 0 || k >= static_cast<int>(ch0.size())) continue;
                const float x = src - static_cast<float>(k);
                const float sinc = (std::fabs(x) < 1e-6f)
                    ? 1.0f
                    : std::sin(static_cast<float>(M_PI) * x)
                        / (static_cast<float>(M_PI) * x);
                const float w = 0.5f * (1.0f + std::cos(
                    static_cast<float>(M_PI) * x / kSincHalf));
                v += ch0[k] * sinc * w;
            }
            ch1[i] = v;
        }

        bool ready = false;
        for (int off = 0; off < total_samples; off += cfg.frame_size) {
            if (loc.Process(ch0_trim.data() + off, ch1.data() + off,
                            cfg.frame_size)) ready = true;
        }
        if (!ready) {
            std::printf("  %6.1f° → NO RESULT\n", ReportAngle2ch(angle, flip));
            ++fail; continue;
        }

        const float expected = ReportAngle2ch(angle, flip);
        const float est = ReportAngle2ch(loc.GetDOA(), flip);
        const float err = std::fabs(est - expected);
        float err_pct = err;
        if (expected > 1.0f && expected < 179.0f)
            err_pct = err / std::min(expected, 180.0f - expected) * 100.0f;

        const bool ok = (err_pct < 3.0f) || (err < 2.5f);
        if (ok) {
            ++pass;
        } else {
            ++fail;
        }
        std::printf("  %6.1f° → %7.2f° | err: %.2f° (%.1f%%) | conf: %.3f | delay: %.3f samp | %s\n",
                    expected, est, err, err_pct, loc.GetConfidence(),
                    delay_samples, ok ? "PASS" : "FAIL");
    }

    std::printf("\n--- Results: %d/%d passed ---\n", pass, pass + fail);
    return fail > 0 ? 1 : 0;
}

int RunFile_2ch(const char* path, float mic_distance, bool verbose, bool flip) {
    WavData wav;
    if (!ReadWav(path, wav)) return 1;
    if (wav.channels != 2) {
        std::fprintf(stderr, "WAV must be stereo (got %d channels). "
            "For multi-channel WAV use -c 3 (or omit -c to auto-detect).\n",
            wav.channels);
        return 1;
    }

    std::printf("=== 2-ch WAV File: %s ===\n", path);
    std::printf("Sample rate: %d Hz | Samples: %zu | Duration: %.2f s\n",
                wav.sample_rate, wav.samples.size() / 2,
                static_cast<float>(wav.samples.size() / 2) / wav.sample_rate);
    if (flip) std::printf("Angle mapping: flipped\n");

    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = wav.sample_rate;
    cfg.mic_distance = mic_distance;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const size_t n_total = wav.samples.size() / 2;
    if (n_total == 0) {
        std::fprintf(stderr, "WAV contains no audio frames\n");
        return 1;
    }
    const float dur = static_cast<float>(n_total) / wav.sample_rate;
    size_t off = 0; int fno = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (off < n_total) {
        size_t chunk = std::min(static_cast<size_t>(cfg.frame_size), n_total - off);
        if (loc.Process(wav.samples.data() + off * 2, chunk)) {
            ++fno;
            if (verbose) {
                if (loc.IsValid()) {
                    const float max_tdoa = cfg.mic_distance / cfg.sound_speed;
                    const float ratio = std::fabs(loc.GetTDOA()) / max_tdoa;
                    std::printf("[Frame %03d] DOA: %7.2f° | TDOA: %+.6fs | Ratio: %.3f | Conf: %.3f\n",
                                fno, ReportAngle2ch(loc.GetDOA(), flip),
                                loc.GetTDOA(), ratio, loc.GetConfidence());
                } else {
                    std::printf("[Frame %03d] DOA: ------- | Conf: %.3f (below threshold)\n",
                                fno, loc.GetConfidence());
                }
            }
        }
        off += chunk;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    if (loc.GetResultCount() > 0)
        std::printf("\nest angle = %.1f\n", ReportAngle2ch(loc.GetAverageDOA(), flip));
    else
        std::printf("\nest angle = N/A (no valid frames)\n");
    std::printf("Process: %.1f ms | Audio: %.2f s | RTF: %.4f\n",
                ms, dur, (ms / 1000.0f) / dur);
    return 0;
}

#ifdef HAS_SPACEMIT_AUDIO
int RunLive_2ch(float mic_distance, int sample_rate, int device_index,
                bool verbose, bool flip) {
    std::printf("=== 2-ch Live Capture ===\n");
    SpacemitAudio::SoundLocatorConfig cfg;
    cfg.sample_rate = sample_rate;
    cfg.mic_distance = mic_distance;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    std::printf("Mic distance: %.4f m | Sample rate: %d Hz | Device: %d\n",
                mic_distance, sample_rate, device_index);

    SpacemitAudio::SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const int chunk = cfg.frame_size * 2 * 2;
    SpacemitAudio::Init(sample_rate, 2, chunk, device_index, -1);
    SpacemitAudio::AudioCapture cap(device_index);
    std::atomic<int> cb{0}, rdy{0}, vld{0};
    std::mutex res_mu;
    float last_doa = 0.0f;
    float last_confidence = 0.0f;
    int last_rdy = 0;
    bool last_valid = false;
    bool have_result = false;

    cap.SetCallback([&](const uint8_t* d, size_t s) {
        ++cb;
        size_t nf = s / (2 * sizeof(int16_t));
        const auto* pcm = reinterpret_cast<const int16_t*>(d);
        if (loc.Process(pcm, nf)) {
            ++rdy;
            const bool valid = loc.IsValid();
            if (valid) {
                ++vld;
            }
            {
                std::lock_guard<std::mutex> lk(res_mu);
                last_doa = ReportAngle2ch(loc.GetDOA(), flip);
                last_confidence = loc.GetConfidence();
                last_rdy = rdy.load();
                last_valid = valid;
                have_result = true;
            }
        }
    });
    if (!cap.Start(sample_rate, 2, chunk)) {
        std::fprintf(stderr, "Failed to start capture\n"); return 1;
    }
    std::printf("Press Ctrl+C to stop.\n\n");
    g_stop_requested = 0;
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        float doa = 0.0f;
        float confidence = 0.0f;
        int idx = 0;
        bool valid = false;
        bool ok = false;
        {
            std::lock_guard<std::mutex> lk(res_mu);
            doa = last_doa;
            confidence = last_confidence;
            idx = last_rdy;
            valid = last_valid;
            ok = have_result;
        }
        if (!ok) continue;
        if (valid) {
            std::printf("\r[%04d] DOA: %7.2f° | Conf: %.3f    ",
                        idx, doa, confidence);
            std::fflush(stdout);
        } else if (verbose) {
            std::printf("\r[%04d] DOA: ------- | Conf: %.3f    ",
                        idx, confidence);
            std::fflush(stdout);
        }
    }
    cap.Stop();
    std::printf("\nStopped. callbacks: %d, frames: %d, valid: %d\n",
                cb.load(), rdy.load(), vld.load());
    return 0;
}
#endif

// ===========================================================================
// 3+ channel mode (MultiSoundLocator)
// ===========================================================================

float ToleranceFor(float a, bool noise) {
    (void)a;
    (void)noise;
    return 5.0f;
}

std::vector<double> MakeSyntheticPhases() {
    std::vector<double> phases;
    phases.reserve(kSyntheticToneCount);
    uint32_t state = static_cast<uint32_t>(kSyntheticSourceSeed);
    for (int i = 0; i < kSyntheticToneCount; ++i) {
        state = state * 1664525u + 1013904223u;
        const double u = static_cast<double>(state) / 4294967296.0;
        phases.push_back(u * 2.0 * M_PI);
    }
    return phases;
}

std::vector<float> GenerateSyntheticInterleaved(
    const SpacemitAudio::MultiSoundLocatorConfig& cfg,
    float angle_deg, float noise_snr_db) {
    const int n_frames = cfg.frame_size * cfg.avg_frames;
    const int sr = cfg.sample_rate;
    const double a_rad =
        static_cast<double>(angle_deg) * M_PI / 180.0;
    const double dx = std::cos(a_rad);
    const double dy = std::sin(a_rad);
    const auto phases = MakeSyntheticPhases();
    std::vector<std::vector<float>> chs;
    chs.reserve(cfg.microphones.size());
    for (const auto& m : cfg.microphones) {
        const double adv =
            (static_cast<double>(m.x) * dx + static_cast<double>(m.y) * dy)
            / static_cast<double>(cfg.sound_speed);
        std::vector<float> channel(n_frames, 0.0f);
        for (int frame = 0; frame < n_frames; ++frame) {
            const double t = static_cast<double>(frame) / static_cast<double>(sr);
            double sample = 0.0;
            for (int tone = 0; tone < kSyntheticToneCount; ++tone) {
                const double ratio = kSyntheticToneCount > 1
                    ? static_cast<double>(tone)
                        / static_cast<double>(kSyntheticToneCount - 1)
                    : 0.0;
                const double freq =
                    kSyntheticMinHz + (kSyntheticMaxHz - kSyntheticMinHz) * ratio;
                sample += std::sin(2.0 * M_PI * freq * (t + adv)
                                    + phases[static_cast<size_t>(tone)]);
            }
            channel[static_cast<size_t>(frame)] =
                static_cast<float>(0.3 * sample
                    / static_cast<double>(kSyntheticToneCount));
        }
        chs.push_back(std::move(channel));
    }
    const bool noise = std::isfinite(noise_snr_db);
    if (noise) {
        std::mt19937 rng(42);
        for (auto& ch : chs) {
            float p = 0.0f; for (float s : ch) p += s * s;
            p /= static_cast<float>(ch.size());
            const float rms = std::sqrt(std::max(p, 1e-12f));
            const float ns = rms / std::pow(10.0f, noise_snr_db / 20.0f);
            std::normal_distribution<float> nz(0.0f, ns);
            for (auto& s : ch) s += nz(rng);
        }
    }
    std::vector<float> il(n_frames * cfg.microphones.size());
    for (int f = 0; f < n_frames; ++f)
        for (size_t c = 0; c < cfg.microphones.size(); ++c)
            il[f * cfg.microphones.size() + c] = chs[c][f];
    return il;
}

bool ParseSweep(const std::string& spec, std::vector<float>& angles) {
    const size_t a = spec.find(':');
    const size_t b = (a == std::string::npos) ? a : spec.find(':', a + 1);
    if (a == std::string::npos || b == std::string::npos) return false;
    const float st = std::stof(spec.substr(0, a));
    const float ed = std::stof(spec.substr(a + 1, b - a - 1));
    const float sp = std::stof(spec.substr(b + 1));
    if (std::fabs(sp) < 1e-6f) return false;
    angles.clear();
    if (sp > 0.0f) {
        for (float v = st; v <= ed + 1e-4f; v += sp) angles.push_back(NormalizeAngle360(v));
    } else {
        for (float v = st; v >= ed - 1e-4f; v += sp) angles.push_back(NormalizeAngle360(v));
    }
    return !angles.empty();
}

bool ParsePositions(const std::string& spec,
                    std::vector<SpacemitAudio::MicrophonePosition>& positions) {
    positions.clear();
    size_t start = 0;
    while (start < spec.size()) {
        const size_t semi = spec.find(';', start);
        const std::string mic = spec.substr(start, semi - start);
        if (mic.empty()) {
            if (semi == std::string::npos) break;
            start = semi + 1; continue;
        }
        float vals[3] = {0.0f, 0.0f, 0.0f}; int n = 0; size_t cs = 0;
        while (cs < mic.size() && n < 3) {
            const size_t comma = mic.find(',', cs);
            const std::string coord = mic.substr(cs, comma - cs);
            if (!coord.empty()) vals[n++] = static_cast<float>(std::atof(coord.c_str()));
            if (comma == std::string::npos) break;
            cs = comma + 1;
        }
        if (n < 2) return false;
        positions.push_back({vals[0], vals[1], vals[2]});
        if (semi == std::string::npos) break;
        start = semi + 1;
    }
    return positions.size() >= 2;
}

struct MultiOverrides {
    bool have_closure = false;     float closure_threshold_samples = 0;
    bool have_closure_fraction = false;
    float closure_threshold_fraction = 0;
    bool have_quality = false;     float quality_threshold = 0;
    bool have_margin = false;      float margin_threshold = 0;
    bool have_min_signal_rms = false; float min_signal_rms = 0;
    bool have_max_freq = false;    float max_frequency_hz = 0;
};

SpacemitAudio::MultiSoundLocatorConfig MakeMultiConfig(
    float mic_distance, int sample_rate, float az_offset, float max_avg_seconds,
    const std::vector<SpacemitAudio::MicrophonePosition>& positions_override,
    const MultiOverrides& ov) {
    SpacemitAudio::MultiSoundLocatorConfig cfg;
    if (positions_override.empty())
        cfg = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(mic_distance);
    else
        cfg.microphones = positions_override;
    cfg.sample_rate = sample_rate;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    cfg.confidence_threshold = 0.1f;
    cfg.azimuth_offset_deg = az_offset;
    cfg.max_avg_seconds = max_avg_seconds;
    if (ov.have_closure)  cfg.closure_threshold_samples = ov.closure_threshold_samples;
    if (ov.have_closure_fraction) {
        cfg.closure_threshold_fraction = ov.closure_threshold_fraction;
    }
    if (ov.have_quality)  cfg.quality_threshold = ov.quality_threshold;
    if (ov.have_margin)   cfg.margin_threshold = ov.margin_threshold;
    if (ov.have_min_signal_rms) cfg.min_signal_rms = ov.min_signal_rms;
    if (ov.have_max_freq) cfg.max_frequency_hz = ov.max_frequency_hz;
    return cfg;
}

int RunTest_3ch(float mic_distance, int sample_rate,
                const std::vector<float>& angles, float az_offset,
                float noise_snr_db, float max_avg_seconds,
                const std::vector<SpacemitAudio::MicrophonePosition>& positions_override,
                const MultiOverrides& ov) {
    std::printf("=== 3+ch Synthetic Accuracy Test ===\n");
    std::printf("Geometry: equilateral | side: %.4f m | sample rate: %d Hz\n",
                mic_distance, sample_rate);
    if (std::isfinite(noise_snr_db)) std::printf("Noise SNR: %.1f dB\n", noise_snr_db);
    std::printf("azimuth_offset_deg: %.2f\n\n", az_offset);
    std::printf(" expected |      est |    err |   conf |  qual | margin | clos.us | pairs | status\n");

    int pass = 0, fail = 0;
    const bool noise = std::isfinite(noise_snr_db);

    if (angles.size() > 1) {
        auto warm_cfg = MakeMultiConfig(mic_distance, sample_rate, az_offset,
            max_avg_seconds, positions_override, ov);
        SpacemitAudio::MultiSoundLocator warm_loc(warm_cfg);
        if (warm_loc.Initialize()) {
            const auto warm = GenerateSyntheticInterleaved(
                warm_cfg, angles.front(), noise_snr_db);
            const int warm_ch = static_cast<int>(warm_cfg.microphones.size());
            warm_loc.Process(warm.data(), warm.size() / warm_ch, warm_ch);
        }
    }

    for (float angle : angles) {
        const float expected = NormalizeAngle360(angle + az_offset);
        auto cfg = MakeMultiConfig(mic_distance, sample_rate, az_offset,
            max_avg_seconds, positions_override, ov);
        SpacemitAudio::MultiSoundLocator loc(cfg);
        if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

        const auto il = GenerateSyntheticInterleaved(cfg, angle, noise_snr_db);
        const int n_ch = static_cast<int>(cfg.microphones.size());
        const size_t n_frames = il.size() / n_ch;
        const bool ready = loc.Process(il.data(), n_frames, n_ch);
        const auto r = loc.GetResult();
        const float err = CircularError(r.azimuth_deg, expected);
        const float tol = ToleranceFor(angle, noise);
        const bool ok = ready && r.valid && err <= tol;
        if (ok) {
            ++pass;
        } else {
            ++fail;
        }
        std::printf(" %8.2f | %8.2f | %6.2f | %6.3f | %5.3f | %6.3f | %7.2f | %5d | %s\n",
                    expected, r.azimuth_deg, err, r.confidence, r.quality,
                    r.score_margin, r.closure_residual_sec * 1e6f,
                    r.valid_pairs, ok ? "PASS" : "FAIL");
    }
    std::printf("\n--- Results: %d/%d passed ---\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}

int RunFile_3ch(const char* path, float mic_distance, bool verbose,
                float az_offset, float max_avg_seconds,
                const std::vector<SpacemitAudio::MicrophonePosition>& positions_override,
                const MultiOverrides& ov) {
    WavData wav;
    if (!ReadWav(path, wav)) return 1;
    if (wav.channels < 3) {
        std::fprintf(stderr, "WAV must have >= 3 channels for -c 3+ (got %d). "
            "For stereo use -c 2.\n", wav.channels);
        return 1;
    }

    auto cfg = MakeMultiConfig(mic_distance, wav.sample_rate, az_offset,
        max_avg_seconds, positions_override, ov);
    if (static_cast<int>(cfg.microphones.size()) != wav.channels) {
        std::fprintf(stderr,
            "WAV has %d channels but config has %d mic positions; "
            "pass --positions or use a matching equilateral side.\n",
            wav.channels, static_cast<int>(cfg.microphones.size()));
        return 1;
    }

    SpacemitAudio::MultiSoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const size_t n_total = wav.samples.size() / wav.channels;
    if (n_total == 0) {
        std::fprintf(stderr, "WAV contains no audio frames\n");
        return 1;
    }
    const float dur = static_cast<float>(n_total) / wav.sample_rate;
    std::printf("=== %dch WAV File: %s ===\n", wav.channels, path);
    std::printf("Sample rate: %d Hz | Frames: %zu | Duration: %.2f s\n",
                wav.sample_rate, n_total, dur);

    size_t off = 0; int fno = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (off < n_total) {
        const size_t chunk = std::min(static_cast<size_t>(cfg.frame_size),
            n_total - off);
        if (loc.Process(wav.samples.data() + off * wav.channels, chunk,
                        wav.channels)) {
            ++fno;
            if (verbose) {
                const auto r = loc.GetResult();
                if (r.valid) {
                    std::printf(
                        "[Frame %03d] az: %7.2f° | conf: %.3f | qual: %.3f | "
                        "clos: %.2f us | pairs: %d\n",
                        fno, r.azimuth_deg, r.confidence, r.quality,
                        r.closure_residual_sec * 1e6f, r.valid_pairs);
                } else {
                    std::printf("[Frame %03d] az: ------- | conf: %.3f\n",
                        fno, r.confidence);
                }
            }
        }
        off += chunk;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    if (loc.GetResultCount() > 0)
        std::printf("\nest azimuth = %.1f (R = %.3f)\n",
                    loc.GetAverageAzimuth(), loc.GetAverageResultantLength());
    else
        std::printf("\nest azimuth = N/A (no valid frames)\n");
    std::printf("Process: %.1f ms | Audio: %.2f s | RTF: %.4f\n",
                ms, dur, (ms / 1000.0f) / dur);
    return 0;
}

#ifdef HAS_SPACEMIT_AUDIO
int RunLive_3ch(float mic_distance, int sample_rate, int device_index,
                bool verbose, float az_offset, float max_avg_seconds,
                const std::vector<SpacemitAudio::MicrophonePosition>& positions_override,
                const MultiOverrides& ov,
                int capture_channels, const std::vector<int>& pick) {
    auto cfg = MakeMultiConfig(mic_distance, sample_rate, az_offset,
        max_avg_seconds, positions_override, ov);
    if (!ov.have_min_signal_rms) {
        cfg.min_signal_rms = kDefaultLiveMinSignalRms;
    }
    const int n_ch = static_cast<int>(cfg.microphones.size());
    if (capture_channels < 0) {
        std::fprintf(stderr,
            "Error: --capture-channels must be >= 0 (got %d)\n",
            capture_channels);
        return 1;
    }
    const int cap_ch = capture_channels > 0 ? capture_channels : n_ch;
    if (cap_ch < n_ch) {
        std::fprintf(stderr,
            "Error: --capture-channels %d < %d DOA mics\n", cap_ch, n_ch);
        return 1;
    }

    // sel_idx is 0-based into the captured frame; sel_idx[m] feeds DOA mic m.
    // Empty --pick = identity, only valid when device opens exactly n_ch chs.
    std::vector<int> sel_idx;
    if (pick.empty()) {
        if (cap_ch != n_ch) {
            std::fprintf(stderr,
                "Error: --capture-channels %d != %d DOA mics requires --pick "
                "(e.g. --pick 2,3,4 to drop an AEC ref on ch1)\n", cap_ch, n_ch);
            return 1;
        }
        for (int m = 0; m < n_ch; ++m) sel_idx.push_back(m);
    } else {
        if (static_cast<int>(pick.size()) != n_ch) {
            std::fprintf(stderr,
                "Error: --pick has %zu entries, need exactly %d (one per DOA mic)\n",
                pick.size(), n_ch);
            return 1;
        }
        for (int v : pick) {
            if (v < 1 || v > cap_ch) {
                std::fprintf(stderr,
                    "Error: --pick value %d out of range [1, %d]\n", v, cap_ch);
                return 1;
            }
            sel_idx.push_back(v - 1);
        }
        std::vector<bool> seen(static_cast<size_t>(cap_ch), false);
        for (int idx : sel_idx) {
            if (seen[static_cast<size_t>(idx)]) {
                std::fprintf(stderr,
                    "Error: --pick repeats capture channel %d\n", idx + 1);
                return 1;
            }
            seen[static_cast<size_t>(idx)] = true;
        }
    }
    const bool identity = (cap_ch == n_ch) && pick.empty();

    SpacemitAudio::MultiSoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const int chunk = cfg.frame_size * cap_ch * sizeof(int16_t);
    SpacemitAudio::Init(sample_rate, cap_ch, chunk, device_index, -1);
    std::atomic<int> cb{0}, rdy{0}, vld{0};
    std::vector<int16_t> sel;  // persistent deinterleave buffer (non-identity)
    if (!identity) {
        sel.resize(static_cast<size_t>(cfg.frame_size)
            * static_cast<size_t>(n_ch));
    }

    // Real-time rule: the audio callback must not block on console I/O. A slow
    // tty (ssh pty) write inside it stalls the ALSA read, overruns the capture
    // buffer, and this USB gadget cannot recover from the xrun. The callback
    // only snapshots the latest result under a tiny lock; the main thread
    // prints it at ~10 Hz.
    std::mutex res_mu;
    SpacemitAudio::MultiSoundLocatorResult last_r{};
    int last_rdy = 0;
    bool have_r = false;
    std::mutex err_mu;
    char callback_error[256] = "";
    std::atomic<bool> callback_failed{false};
    std::atomic<int> callback_errors{0};

    auto record_callback_error = [&](const char* msg) {
        callback_failed.store(true, std::memory_order_relaxed);
        if (callback_errors.fetch_add(1, std::memory_order_relaxed) == 0) {
            std::lock_guard<std::mutex> lk(err_mu);
            std::snprintf(callback_error, sizeof(callback_error), "%s",
                msg ? msg : "unknown callback error");
        }
    };

    // Construct capture after callback state so destructor order stops/closes
    // the stream before any referenced callback state is destroyed.
    SpacemitAudio::AudioCapture cap(device_index);
    cap.SetCallback([&](const uint8_t* d, size_t s) {
        if (callback_failed.load(std::memory_order_relaxed)) return;
        try {
            ++cb;
            const size_t nf = s / (cap_ch * sizeof(int16_t));
            if (nf == 0) return;
            const auto* pcm = reinterpret_cast<const int16_t*>(d);
            const int16_t* feed = pcm;
            if (!identity) {
                const size_t needed = nf * static_cast<size_t>(n_ch);
                if (sel.size() < needed) {
                    record_callback_error("capture callback exceeded preallocated frame buffer");
                    return;
                }
                for (size_t f = 0; f < nf; ++f) {
                    for (int m = 0; m < n_ch; ++m) {
                        sel[f * n_ch + m] = pcm[f * cap_ch + sel_idx[m]];
                    }
                }
                feed = sel.data();
            }
            if (loc.Process(feed, nf, n_ch)) {
                ++rdy;
                const auto r = loc.GetResult();
                if (r.valid) ++vld;
                {
                    std::lock_guard<std::mutex> lk(res_mu);
                    last_r = r;
                    last_rdy = rdy.load();
                    have_r = true;
                }
            }
        } catch (const std::exception& e) {
            record_callback_error(e.what());
        } catch (...) {
            record_callback_error("unknown callback exception");
        }
    });
    if (!cap.Start(sample_rate, cap_ch, chunk)) {
        std::fprintf(stderr, "Failed to start capture\n"); return 1;
    }
    std::printf("=== Live Capture ===\ncapture: %dch | DOA mics: %d | side: %.4f m | "
                "rate: %d Hz | device: %d\n",
                cap_ch, n_ch, mic_distance, sample_rate, device_index);
    if (!identity) {
        std::printf("channel map (1-based): ");
        for (int m = 0; m < n_ch; ++m) {
            std::printf("ch%d->mic%d%s", sel_idx[m] + 1, m,
                        m + 1 < n_ch ? ", " : "\n");
        }
    }
    std::printf("Press Ctrl+C to stop.\n\n");
    g_stop_requested = 0;
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    int printed_rdy = 0;
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (callback_failed.load(std::memory_order_relaxed)) {
            char message[sizeof(callback_error)] = "";
            {
                std::lock_guard<std::mutex> lk(err_mu);
                std::snprintf(message, sizeof(message), "%s", callback_error);
            }
            std::fprintf(stderr, "\nAudio callback failed: %s\n", message);
            break;
        }
        SpacemitAudio::MultiSoundLocatorResult r;
        int idx = 0;
        bool ok = false;
        {
            std::lock_guard<std::mutex> lk(res_mu);
            r = last_r;
            idx = last_rdy;
            ok = have_r;
        }
        if (!ok) continue;
        if (idx == printed_rdy) continue;
        printed_rdy = idx;
        if (r.valid) {
            std::printf("\r[%04d] az: %7.2f° | conf: %.3f | qual: %.3f | clos: %.2f us    ",
                        idx, r.azimuth_deg, r.confidence, r.quality,
                        r.closure_residual_sec * 1e6f);
            std::fflush(stdout);
        } else if (verbose) {
            std::printf("\r[%04d] az: ------- | conf: %.3f    ",
                        idx, r.confidence);
            std::fflush(stdout);
        }
    }
    cap.Stop();
    std::printf("\nStopped. callbacks: %d, frames: %d, valid: %d\n",
                cb.load(), rdy.load(), vld.load());
    return 0;
}
#endif

// ===========================================================================
// Usage + main
// ===========================================================================

void PrintUsage(const char* prog) {
    std::printf("Usage (channels selected by -c N; N=2 → SoundLocator, N>=3 → MultiSoundLocator):\n");
    std::printf("  %s -c 2 -t [-d M] [-r RATE] [--flip]\n", prog);
    std::printf("  %s -c 2 -f stereo.wav [-d M] [-v] [--flip]\n", prog);
#ifdef HAS_SPACEMIT_AUDIO
    std::printf("  %s -c 2 -l [-d M] [-r RATE] [-i DEV] [--flip]\n", prog);
#endif
    std::printf("\n");
    std::printf("  %s -c 3 -t [-d M] [--angle DEG | --sweep A:B:S] [--noise-snr DB]\n", prog);
    std::printf("                   [--azimuth-offset DEG] [--positions SPEC]\n");
    std::printf("                   [--avg-seconds S] [--quality-threshold F]\n");
    std::printf("                   [--margin-threshold F] [--closure-threshold-samples F]\n");
    std::printf("                   [--closure-threshold-fraction F] [--min-signal-rms F]\n");
    std::printf("                   [--max-frequency-hz F]\n");
    std::printf("  %s -c 3 -f Nch.wav [-d M] [-v] [--positions SPEC]\n", prog);
    std::printf("                   [--azimuth-offset DEG] [--avg-seconds S]\n");
#ifdef HAS_SPACEMIT_AUDIO
    std::printf("  %s -c 3 -l [-d M] [-r RATE] [-i DEV] [-v]\n", prog);
    std::printf("                   [--positions SPEC] [--azimuth-offset DEG]\n");
    std::printf("                   [--avg-seconds S] [--min-signal-rms F]\n");
    std::printf("                   [--capture-channels N] [--pick i,j,k]\n");
#endif
    std::printf("\n");
    std::printf("  %s -h | --help\n\n", prog);
    std::printf("Common flags:\n");
    std::printf("  -c N                       Channel count (2 → 2-ch, >=3 → multi). In -f mode\n");
    std::printf("                              you can omit -c and it auto-detects from the WAV.\n");
    std::printf("  -d, --mic-distance M       Mic spacing in meters. Default %.4f (2-ch) /\n",
                kDefault2chMicDistance);
    std::printf("                              %.4f (3-ch). A WARNING is printed if you don't\n",
                kDefault3chMicDistance);
    std::printf("                              set it explicitly — production code MUST measure\n");
    std::printf("                              the physical spacing.\n");
    std::printf("  -r RATE                    Sample rate Hz (default 16000)\n");
    std::printf("  -i DEV                     Capture device index (default -1 = auto)\n");
    std::printf("  -v, --verbose              Per-frame output (file / live mode)\n");
    std::printf("  -t / -f WAV / -l           Mode selector\n");
    std::printf("\n2-ch (-c 2) specific:\n");
    std::printf("  --flip                     Report 180 − DOA (legacy alias). For -c 3 use\n");
    std::printf("                              --azimuth-offset instead.\n");
    std::printf("\n3+ ch (-c 3+) specific:\n");
    std::printf("  --angle DEG                Single synthetic angle (test mode)\n");
    std::printf("  --sweep A:B:S              Synthetic sweep, inclusive (test mode)\n");
    std::printf("  --noise-snr DB             Add Gaussian noise at the requested SNR\n");
    std::printf("  --azimuth-offset DEG       Array → robot frame offset (degrees)\n");
    std::printf("  --positions x0,y0;x1,y1... Override mic positions (N>=2; skips equilateral)\n");
    std::printf("  --avg-seconds S            Sliding-window length for GetAverageAzimuth.\n");
    std::printf("                              Default: 0 for test/file, %.0fs for live.\n",
                kDefaultLiveAvgSeconds);
    std::printf("                              Pass 0 explicitly in live mode for unbounded\n");
    std::printf("                              history since Reset.\n");
    std::printf("  --quality-threshold F      Override config.quality_threshold (default 0.0)\n");
    std::printf("  --margin-threshold F       Override config.margin_threshold (default 0.6)\n");
    std::printf("  --min-signal-rms F         Drop frames below this RMS before GCC-PHAT.\n");
    std::printf("                              Default: 0 for test/file, %.3f for live.\n",
                kDefaultLiveMinSignalRms);
    std::printf("                              Pass 0 in live mode to disable this activity gate.\n");
    std::printf("  --closure-threshold-samples F\n");
    std::printf("                              Explicit closure cap in samples (N=3 only;\n");
    std::printf("                              default 0.0; effective=max(samples,\n");
    std::printf("                              fraction*physical_max))\n");
    std::printf("  --closure-threshold-fraction F\n");
    std::printf("                              Closure cap as fraction of max physical TDOA\n");
    std::printf("                              (N=3 only; default 0.3). Set both closure\n");
    std::printf("                              thresholds <=0 to disable the gate.\n");
    std::printf("  --max-frequency-hz F       Override config.max_frequency_hz (default 0 =\n");
    std::printf("                              auto alias-safe c / 2·d_max)\n");
#ifdef HAS_SPACEMIT_AUDIO
    std::printf("\nLive (-l) channel selection (e.g. 4-mic: ch1=AEC ref, ch2-4=DOA):\n");
    std::printf("  --capture-channels N       Open N device channels (default = DOA mic\n");
    std::printf("                              count). Use when the device exposes extra\n");
    std::printf("                              channels (AEC ref / loopback).\n");
    std::printf("  --pick i,j,k               1-based capture channels feeding DOA mics,\n");
    std::printf("                              one per mic. Required when --capture-channels\n");
    std::printf("                              != mic count. e.g. --capture-channels 4\n");
    std::printf("                              --pick 2,3,4 drops the ch1 AEC reference.\n");
#endif
}

struct Args {
    int channels = 0;
    enum Mode { NONE, TEST, FILE, LIVE } mode = NONE;
    const char* wav_path = nullptr;
    float mic_distance = 0.0f;
    bool mic_distance_set = false;
    int sample_rate = 16000;
    int device_index = -1;
    bool verbose = false;
    bool flip = false;
    bool have_angle = false;
    float angle = 90.0f;
    std::string sweep_spec;
    float az_offset = 0.0f;
    float noise_snr_db = std::numeric_limits<float>::infinity();
    float max_avg_seconds = 0.0f;
    bool avg_seconds_set = false;
    std::vector<SpacemitAudio::MicrophonePosition> positions_override;
    MultiOverrides multi_overrides;
    int capture_channels = 0;       // 0 = same as DOA mic count (current behavior)
    std::vector<int> pick;          // 1-based capture-channel -> DOA mic map; empty = identity
    bool help = false;
};

bool ParseArgs(int argc, char* argv[], Args& a) {
    for (int i = 1; i < argc; ++i) {
        const char* s = argv[i];
        if (std::strcmp(s, "-h") == 0 || std::strcmp(s, "--help") == 0) {
            a.help = true;
        } else if (std::strcmp(s, "-c") == 0 && i + 1 < argc) {
            a.channels = std::atoi(argv[++i]);
        } else if (std::strcmp(s, "-t") == 0 || std::strcmp(s, "--test") == 0) {
            a.mode = Args::TEST;
        } else if (std::strcmp(s, "-f") == 0 || std::strcmp(s, "--file") == 0) {
            a.mode = Args::FILE;
            if (i + 1 < argc && argv[i + 1][0] != '-') a.wav_path = argv[++i];
        } else if (std::strcmp(s, "-l") == 0 || std::strcmp(s, "--live") == 0) {
            a.mode = Args::LIVE;
        } else if ((std::strcmp(s, "-d") == 0 || std::strcmp(s, "--mic-distance") == 0)
            && i + 1 < argc) {
            a.mic_distance = static_cast<float>(std::atof(argv[++i]));
            a.mic_distance_set = true;
        } else if (std::strcmp(s, "-r") == 0 && i + 1 < argc) {
            a.sample_rate = std::atoi(argv[++i]);
        } else if (std::strcmp(s, "-i") == 0 && i + 1 < argc) {
            a.device_index = std::atoi(argv[++i]);
        } else if (std::strcmp(s, "-v") == 0 || std::strcmp(s, "--verbose") == 0) {
            a.verbose = true;
        } else if (std::strcmp(s, "--flip") == 0) {
            a.flip = true;
        } else if (std::strcmp(s, "--angle") == 0 && i + 1 < argc) {
            a.angle = static_cast<float>(std::atof(argv[++i])); a.have_angle = true;
        } else if (std::strcmp(s, "--sweep") == 0 && i + 1 < argc) {
            a.sweep_spec = argv[++i];
        } else if (std::strcmp(s, "--noise-snr") == 0 && i + 1 < argc) {
            a.noise_snr_db = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--azimuth-offset") == 0 && i + 1 < argc) {
            a.az_offset = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--avg-seconds") == 0 && i + 1 < argc) {
            a.max_avg_seconds = static_cast<float>(std::atof(argv[++i]));
            a.avg_seconds_set = true;
        } else if (std::strcmp(s, "--positions") == 0 && i + 1 < argc) {
            if (!ParsePositions(argv[++i], a.positions_override)) {
                std::fprintf(stderr, "Error: --positions must be N>=2 mics 'x,y[,z];...'\n");
                return false;
            }
        } else if (std::strcmp(s, "--quality-threshold") == 0 && i + 1 < argc) {
            a.multi_overrides.have_quality = true;
            a.multi_overrides.quality_threshold = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--margin-threshold") == 0 && i + 1 < argc) {
            a.multi_overrides.have_margin = true;
            a.multi_overrides.margin_threshold = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--min-signal-rms") == 0 && i + 1 < argc) {
            a.multi_overrides.have_min_signal_rms = true;
            a.multi_overrides.min_signal_rms = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--closure-threshold-samples") == 0 && i + 1 < argc) {
            a.multi_overrides.have_closure = true;
            a.multi_overrides.closure_threshold_samples =
                static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--closure-threshold-fraction") == 0
            && i + 1 < argc) {
            a.multi_overrides.have_closure_fraction = true;
            a.multi_overrides.closure_threshold_fraction =
                static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--max-frequency-hz") == 0 && i + 1 < argc) {
            a.multi_overrides.have_max_freq = true;
            a.multi_overrides.max_frequency_hz = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(s, "--capture-channels") == 0 && i + 1 < argc) {
            a.capture_channels = std::atoi(argv[++i]);
        } else if (std::strcmp(s, "--pick") == 0 && i + 1 < argc) {
            a.pick.clear();
            for (const char* p = argv[++i]; *p != '\0';) {
                a.pick.push_back(std::atoi(p));
                while (*p != '\0' && *p != ',') ++p;
                if (*p == ',') ++p;
            }
        } else if (s[0] == '-') {
            std::fprintf(stderr, "Unknown flag: %s\n", s);
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    Args a;
    if (!ParseArgs(argc, argv, a)) {
        PrintUsage(argv[0]);
        return 1;
    }
    if (a.help) {
        PrintUsage(argv[0]);
        return 0;
    }
    if (a.mode == Args::NONE) {
        std::fprintf(stderr, "Error: choose a mode: -t / -f WAV / -l\n\n");
        PrintUsage(argv[0]);
        return 1;
    }

    // Auto-detect channels from WAV in file mode if -c was omitted.
    if (a.channels == 0 && a.mode == Args::FILE && a.wav_path) {
        WavHeaderInfo probe;
        if (!ReadWavHeader(a.wav_path, probe)) return 1;
        a.channels = probe.channels;
        std::printf("[ssl_demo] auto-detected -c %d from WAV header\n", a.channels);
    }
    if (a.channels == 0) a.channels = 2;  // default for synthetic / live without -c

    if (a.channels < 2) {
        std::fprintf(stderr, "Error: -c must be >= 2 (got %d)\n", a.channels);
        return 1;
    }
    if (a.channels >= 3 && a.flip) {
        std::fprintf(stderr,
            "Error: --flip is 2-ch only. For 3+ ch use --azimuth-offset 180.\n");
        return 1;
    }

    // Apply per-mode default mic_distance + warn if not user-set.
    if (a.channels == 2) {
        const float used = a.mic_distance_set ? a.mic_distance : kDefault2chMicDistance;
        WarnIfDefaultMicDistance(a.mic_distance_set, used, "2-ch (stereo) demo");
        a.mic_distance = used;
    } else {
        const float used = a.mic_distance_set ? a.mic_distance : kDefault3chMicDistance;
        WarnIfDefaultMicDistance(a.mic_distance_set, used, "3-ch equilateral demo");
        a.mic_distance = used;
    }

    // Dispatch.
    if (a.channels == 2) {
        switch (a.mode) {
            case Args::TEST:
                return RunTest_2ch(a.mic_distance, a.sample_rate, a.flip);
            case Args::FILE:
                if (!a.wav_path) {
                    std::fprintf(stderr, "Error: -f requires a WAV path\n");
                    return 1;
                }
                return RunFile_2ch(a.wav_path, a.mic_distance, a.verbose, a.flip);
            case Args::LIVE:
#ifdef HAS_SPACEMIT_AUDIO
                return RunLive_2ch(a.mic_distance, a.sample_rate,
                    a.device_index, a.verbose, a.flip);
#else
                std::fprintf(stderr,
                    "Live mode disabled: spacemit_audio not found at build time\n");
                return 1;
#endif
            default: break;
        }
    } else {
        std::vector<float> angles;
        if (a.mode == Args::TEST) {
            if (!a.sweep_spec.empty()) {
                try {
                    if (!ParseSweep(a.sweep_spec, angles)) {
                        std::fprintf(stderr, "Invalid --sweep: %s\n",
                            a.sweep_spec.c_str());
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "Invalid --sweep: %s (%s)\n",
                        a.sweep_spec.c_str(), e.what());
                    return 1;
                }
            } else if (a.have_angle) {
                angles.push_back(NormalizeAngle360(a.angle));
            } else {
                ParseSweep("0:330:30", angles);
            }
        }
        switch (a.mode) {
            case Args::TEST:
                return RunTest_3ch(a.mic_distance, a.sample_rate, angles,
                    a.az_offset, a.noise_snr_db, a.max_avg_seconds,
                    a.positions_override, a.multi_overrides);
            case Args::FILE:
                if (!a.wav_path) {
                    std::fprintf(stderr, "Error: -f requires a WAV path\n");
                    return 1;
                }
                return RunFile_3ch(a.wav_path, a.mic_distance, a.verbose,
                    a.az_offset, a.max_avg_seconds,
                    a.positions_override, a.multi_overrides);
            case Args::LIVE:
#ifdef HAS_SPACEMIT_AUDIO
            {
                const float max_avg_seconds = a.avg_seconds_set
                    ? a.max_avg_seconds : kDefaultLiveAvgSeconds;
                return RunLive_3ch(a.mic_distance, a.sample_rate, a.device_index,
                    a.verbose, a.az_offset, max_avg_seconds,
                    a.positions_override, a.multi_overrides,
                    a.capture_channels, a.pick);
            }
#else
                std::fprintf(stderr,
                    "Live mode disabled: spacemit_audio not found at build time\n");
                return 1;
#endif
            default: break;
        }
    }

    PrintUsage(argv[0]);
    return 1;
}
