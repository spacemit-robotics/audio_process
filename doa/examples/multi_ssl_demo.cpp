/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * multi_ssl_demo — Multi-channel sound source localization demo
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
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "multi_sound_locator.h"

#ifdef HAS_SPACEMIT_AUDIO
#include "audio_base.hpp"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr int kSyntheticChannels = 3;
constexpr int kLanczosTaps = 31;
constexpr int kLanczosHalf = kLanczosTaps / 2;
constexpr float kDefaultMicDistance = 0.063f;

struct WavData {
    std::vector<int16_t> samples;
    int channels = 0;
    int sample_rate = 0;
};

float NormalizeAngle360(float degrees) {
    float value = std::fmod(degrees, 360.0f);
    if (value < 0.0f) {
        value += 360.0f;
    }
    if (value >= 360.0f) {
        value -= 360.0f;
    }
    return value;
}

float CircularError(float a, float b) {
    float diff = std::fabs(NormalizeAngle360(a) - NormalizeAngle360(b));
    return std::min(diff, 360.0f - diff);
}

bool IsPairEndfireAngle(float angle) {
    constexpr float kAngles[] = {60.0f, 120.0f, 240.0f, 300.0f};
    for (float candidate : kAngles) {
        if (CircularError(angle, candidate) <= 0.5f) {
            return true;
        }
    }
    return false;
}

float ToleranceFor(float angle, bool noise_enabled) {
    if (noise_enabled || IsPairEndfireAngle(angle)) {
        return 5.0f;
    }
    return 2.0f;
}

float Sinc(float x) {
    if (std::fabs(x) < 1e-6f) {
        return 1.0f;
    }
    const float pix = static_cast<float>(M_PI) * x;
    return std::sin(pix) / pix;
}

float LanczosKernel(float x) {
    constexpr float a = static_cast<float>(kLanczosHalf);
    if (std::fabs(x) >= a) {
        return 0.0f;
    }
    return Sinc(x) * Sinc(x / a);
}

std::vector<float> MakeChirp(int n_samples, int sample_rate) {
    std::vector<float> signal(n_samples);
    const float duration = static_cast<float>(n_samples) / sample_rate;
    const float f0 = 200.0f;
    const float f1 = 4000.0f;
    const float sweep = (f1 - f0) / duration;

    for (int n = 0; n < n_samples; ++n) {
        const float t = static_cast<float>(n) / sample_rate;
        const float phase = 2.0f * static_cast<float>(M_PI)
            * (f0 * t + 0.5f * sweep * t * t);
        signal[n] = 0.7f * std::sin(phase);
    }
    return signal;
}

std::vector<float> FractionalDelay(
    const std::vector<float>& base, int n_frames, float delay_samples) {
    std::vector<float> out(n_frames, 0.0f);
    const int pad = (static_cast<int>(base.size()) - n_frames) / 2;

    for (int n = 0; n < n_frames; ++n) {
        const float src_pos = static_cast<float>(n + pad) - delay_samples;
        const int center = static_cast<int>(std::floor(src_pos));
        float value = 0.0f;
        float weight_sum = 0.0f;

        for (int k = center - kLanczosHalf; k <= center + kLanczosHalf; ++k) {
            if (k < 0 || k >= static_cast<int>(base.size())) {
                continue;
            }
            const float x = src_pos - static_cast<float>(k);
            const float w = LanczosKernel(x);
            value += base[k] * w;
            weight_sum += w;
        }

        if (std::fabs(weight_sum) > 1e-6f) {
            value /= weight_sum;
        }
        out[n] = value;
    }

    return out;
}

std::vector<float> GenerateSyntheticInterleaved(
    const SpacemitAudio::MultiSoundLocatorConfig& cfg,
    float angle_deg,
    float noise_snr_db) {
    const int n_frames = cfg.frame_size * cfg.avg_frames;
    const int pad = 64;
    const int sample_rate = cfg.sample_rate;
    const float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;
    const float dir_x = std::cos(angle_rad);
    const float dir_y = std::sin(angle_rad);

    auto base = MakeChirp(n_frames + 2 * pad, sample_rate);
    std::vector<std::vector<float>> channels;
    channels.reserve(cfg.microphones.size());

    for (const auto& mic : cfg.microphones) {
        const float advance_seconds = (mic.x * dir_x + mic.y * dir_y)
            / cfg.sound_speed;
        const float delay_samples = -advance_seconds * sample_rate;
        channels.push_back(FractionalDelay(base, n_frames, delay_samples));
    }

    const bool add_noise = std::isfinite(noise_snr_db);
    std::mt19937 rng(42);
    if (add_noise) {
        for (auto& channel : channels) {
            float power = 0.0f;
            for (float sample : channel) {
                power += sample * sample;
            }
            power /= static_cast<float>(channel.size());
            const float rms = std::sqrt(std::max(power, 1e-12f));
            const float noise_std = rms / std::pow(10.0f, noise_snr_db / 20.0f);
            std::normal_distribution<float> noise(0.0f, noise_std);
            for (auto& sample : channel) {
                sample += noise(rng);
            }
        }
    }

    std::vector<float> interleaved(n_frames * cfg.microphones.size());
    for (int frame = 0; frame < n_frames; ++frame) {
        for (size_t ch = 0; ch < cfg.microphones.size(); ++ch) {
            interleaved[frame * cfg.microphones.size() + ch] = channels[ch][frame];
        }
    }
    return interleaved;
}

bool ParseSweep(const std::string& spec, std::vector<float>& angles) {
    const size_t first = spec.find(':');
    const size_t second = spec.find(
        ':', first == std::string::npos ? first : first + 1);
    if (first == std::string::npos || second == std::string::npos) {
        return false;
    }

    const float start = std::stof(spec.substr(0, first));
    const float end = std::stof(spec.substr(first + 1, second - first - 1));
    const float step = std::stof(spec.substr(second + 1));
    if (std::fabs(step) < 1e-6f) {
        return false;
    }

    angles.clear();
    if (step > 0.0f) {
        for (float value = start; value <= end + 1e-4f; value += step) {
            angles.push_back(NormalizeAngle360(value));
        }
    } else {
        for (float value = start; value >= end - 1e-4f; value += step) {
            angles.push_back(NormalizeAngle360(value));
        }
    }
    return !angles.empty();
}

bool ReadWav(const char* path, WavData& wav) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::fprintf(stderr, "Cannot open %s\n", path);
        return false;
    }

    char riff[4];
    uint32_t riff_size = 0;
    char wave[4];
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
    uint16_t audio_format = 0;
    uint16_t channels = 0;
    uint32_t sample_rate = 0;
    uint16_t bits_per_sample = 0;

    while (!std::feof(f)) {
        char chunk_id[4];
        uint32_t chunk_size = 0;
        if (std::fread(chunk_id, 1, 4, f) != 4
            || std::fread(&chunk_size, sizeof(chunk_size), 1, f) != 1) {
            break;
        }

        if (std::memcmp(chunk_id, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                std::fclose(f);
                return false;
            }
            uint32_t byte_rate = 0;
            uint16_t block_align = 0;
            if (std::fread(&audio_format, sizeof(audio_format), 1, f) != 1
                || std::fread(&channels, sizeof(channels), 1, f) != 1
                || std::fread(&sample_rate, sizeof(sample_rate), 1, f) != 1
                || std::fread(&byte_rate, sizeof(byte_rate), 1, f) != 1
                || std::fread(&block_align, sizeof(block_align), 1, f) != 1
                || std::fread(&bits_per_sample,
                              sizeof(bits_per_sample), 1, f) != 1) {
                std::fclose(f);
                std::fprintf(stderr, "Failed to read WAV fmt chunk\n");
                return false;
            }
            if (chunk_size > 16) {
                std::fseek(f, static_cast<long>(chunk_size - 16), SEEK_CUR);
            }
            got_fmt = true;
        } else if (std::memcmp(chunk_id, "data", 4) == 0) {
            if (!got_fmt) {
                std::fclose(f);
                std::fprintf(stderr, "WAV data chunk appeared before fmt chunk\n");
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
            const size_t read_count = std::fread(
                wav.samples.data(), sizeof(int16_t), wav.samples.size(), f);
            std::fclose(f);
            if (read_count != wav.samples.size()) {
                std::fprintf(stderr, "Failed to read WAV data\n");
                return false;
            }
            return true;
        } else {
            std::fseek(f, static_cast<long>(chunk_size), SEEK_CUR);
        }

        if (chunk_size % 2 != 0) {
            std::fseek(f, 1, SEEK_CUR);
        }
    }

    std::fclose(f);
    std::fprintf(stderr, "No data chunk found\n");
    return false;
}

SpacemitAudio::MultiSoundLocatorConfig MakeConfig(
    float mic_distance,
    int sample_rate,
    float azimuth_offset_deg,
    float max_avg_seconds,
    const std::vector<SpacemitAudio::MicrophonePosition>& positions_override) {
    SpacemitAudio::MultiSoundLocatorConfig cfg;
    if (positions_override.empty()) {
        cfg = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(
            mic_distance);
    } else {
        cfg.microphones = positions_override;
    }
    cfg.sample_rate = sample_rate;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    cfg.confidence_threshold = 0.1f;  // restored library default (was 0.02f override; margin_threshold is the real silence gate)
    cfg.azimuth_offset_deg = azimuth_offset_deg;
    cfg.max_avg_seconds = max_avg_seconds;
    return cfg;
}

bool ParsePositions(const std::string& spec,
                    std::vector<SpacemitAudio::MicrophonePosition>& positions) {
    positions.clear();
    size_t start = 0;
    while (start < spec.size()) {
        size_t semi = spec.find(';', start);
        std::string mic = spec.substr(start, semi - start);
        if (mic.empty()) {
            if (semi == std::string::npos) break;
            start = semi + 1;
            continue;
        }
        float vals[3] = {0.0f, 0.0f, 0.0f};
        int n = 0;
        size_t cstart = 0;
        while (cstart < mic.size() && n < 3) {
            size_t comma = mic.find(',', cstart);
            std::string coord = mic.substr(cstart, comma - cstart);
            if (!coord.empty()) {
                vals[n++] = static_cast<float>(std::atof(coord.c_str()));
            }
            if (comma == std::string::npos) break;
            cstart = comma + 1;
        }
        if (n < 2) return false;
        positions.push_back(SpacemitAudio::MicrophonePosition{vals[0], vals[1], vals[2]});
        if (semi == std::string::npos) break;
        start = semi + 1;
    }
    return positions.size() >= 2;
}

int RunSynthetic(
    float mic_distance,
    int sample_rate,
    const std::vector<float>& angles,
    float azimuth_offset_deg,
    float noise_snr_db,
    float max_avg_seconds,
    const std::vector<SpacemitAudio::MicrophonePosition>& positions_override) {
    std::printf("=== Multi-channel Synthetic Accuracy Test ===\n");
    std::printf("Geometry: equilateral | side: %.4f m | sample rate: %d Hz\n",
                mic_distance, sample_rate);
    if (std::isfinite(noise_snr_db)) {
        std::printf("Noise SNR: %.1f dB\n", noise_snr_db);
    }
    std::printf("azimuth_offset_deg: %.2f\n\n", azimuth_offset_deg);
    std::printf(" expected |      est |    err |   conf | margin | pairs | status\n");

    int pass = 0;
    int fail = 0;
    const bool noise_enabled = std::isfinite(noise_snr_db);

    for (float angle : angles) {
        const float expected = NormalizeAngle360(angle + azimuth_offset_deg);
        auto cfg = MakeConfig(mic_distance, sample_rate, azimuth_offset_deg,
                              max_avg_seconds, positions_override);
        SpacemitAudio::MultiSoundLocator loc(cfg);
        if (!loc.Initialize()) {
            std::fprintf(stderr, "Initialize failed\n");
            return 1;
        }

        const auto interleaved = GenerateSyntheticInterleaved(
            cfg, angle, noise_snr_db);
        const size_t n_frames = interleaved.size() / kSyntheticChannels;
        const bool ready = loc.Process(interleaved.data(), n_frames,
                                       kSyntheticChannels);
        const auto result = loc.GetResult();
        const float err = CircularError(result.azimuth_deg, expected);
        const float tolerance = ToleranceFor(angle, noise_enabled);
        const bool ok = ready && result.valid && err <= tolerance;
        if (ok) {
            ++pass;
        } else {
            ++fail;
        }

        std::printf(" %8.2f | %8.2f | %6.2f | %6.3f | %6.3f | %5d | %s\n",
                    expected, result.azimuth_deg, err, result.confidence,
                    result.score_margin, result.valid_pairs,
                    ok ? "PASS" : "FAIL");
    }

    std::printf("\n--- Results: %d/%d passed ---\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}

int RunFile(const char* path, float mic_distance, bool verbose, float max_avg_seconds, const std::vector<SpacemitAudio::MicrophonePosition>& positions_override,
            float azimuth_offset_deg) {
    WavData wav;
    if (!ReadWav(path, wav)) {
        return 1;
    }
    if (wav.channels != kSyntheticChannels) {
        std::fprintf(stderr, "WAV must be 3-channel PCM16 (got %d channels)\n",
                     wav.channels);
        return 1;
    }

    auto cfg = MakeConfig(mic_distance, wav.sample_rate, azimuth_offset_deg,
                          max_avg_seconds, positions_override);
    SpacemitAudio::MultiSoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const size_t n_frames_total = wav.samples.size() / wav.channels;
    const float duration = static_cast<float>(n_frames_total) / wav.sample_rate;
    std::printf("=== 3ch WAV File: %s ===\n", path);
    std::printf("Sample rate: %d Hz | Frames: %zu | Duration: %.2f s\n",
                wav.sample_rate, n_frames_total, duration);

    size_t offset = 0;
    int frame_no = 0;
    const auto t_start = std::chrono::steady_clock::now();

    while (offset < n_frames_total) {
        const size_t chunk = std::min(static_cast<size_t>(cfg.frame_size),
                                      n_frames_total - offset);
        if (loc.Process(wav.samples.data() + offset * wav.channels, chunk,
                        wav.channels)) {
            ++frame_no;
            if (verbose) {
                const auto result = loc.GetResult();
                if (result.valid) {
                    std::printf("[Frame %03d] azimuth: %7.2f deg | conf: %.3f | margin: %.3f | pairs: %d\n",
                                frame_no, result.azimuth_deg, result.confidence,
                                result.score_margin, result.valid_pairs);
                } else {
                    std::printf("[Frame %03d] azimuth: ------- | conf: %.3f\n",
                                frame_no, result.confidence);
                }
            }
        }
        offset += chunk;
    }

    const auto t_end = std::chrono::steady_clock::now();
    const float process_ms = std::chrono::duration<float, std::milli>(
        t_end - t_start).count();
    if (loc.GetResultCount() > 0) {
        std::printf("\nest azimuth = %.1f\n", loc.GetAverageAzimuth());
    } else {
        std::printf("\nest azimuth = N/A (no valid frames)\n");
    }
    std::printf("Process: %.1f ms | Audio: %.2f s | RTF: %.4f\n",
                process_ms, duration, (process_ms / 1000.0f) / duration);
    return 0;
}

#ifdef HAS_SPACEMIT_AUDIO
static volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) {
    g_stop_requested = 1;
}

int RunLive(float mic_distance, int sample_rate, int device_index,
            bool verbose, float azimuth_offset_deg,
            float max_avg_seconds,
            const std::vector<SpacemitAudio::MicrophonePosition>& positions_override) {
    auto cfg = MakeConfig(mic_distance, sample_rate, azimuth_offset_deg);
    SpacemitAudio::MultiSoundLocator loc(cfg);
    if (!loc.Initialize()) {
        std::fprintf(stderr, "Initialize failed\n");
        return 1;
    }

    const int chunk_size = cfg.frame_size * kSyntheticChannels * sizeof(int16_t);
    SpacemitAudio::Init(sample_rate, kSyntheticChannels, chunk_size,
                        device_index, -1);
    SpacemitAudio::AudioCapture capture(device_index);

    std::atomic<int> callbacks{0};
    std::atomic<int> ready_count{0};
    std::atomic<int> valid_count{0};
    std::atomic<int> confidence_milli{0};

    capture.SetCallback([&](const uint8_t* data, size_t size) {
        ++callbacks;
        const size_t frames = size / (kSyntheticChannels * sizeof(int16_t));
        if (frames == 0) {
            return;
        }
        const auto* pcm = reinterpret_cast<const int16_t*>(data);
        if (loc.Process(pcm, frames, kSyntheticChannels)) {
            ++ready_count;
            const auto result = loc.GetResult();
            confidence_milli.store(static_cast<int>(result.confidence * 1000.0f));
            if (result.valid) {
                ++valid_count;
                std::printf("\r[%04d] azimuth: %7.2f deg | conf: %.3f | margin: %.3f    ",
                            ready_count.load(), result.azimuth_deg,
                            result.confidence, result.score_margin);
                std::fflush(stdout);
            } else if (verbose) {
                std::printf("\r[%04d] azimuth: ------- | conf: %.3f    ",
                            ready_count.load(), result.confidence);
                std::fflush(stdout);
            }
        }
    });

    if (!capture.Start(sample_rate, kSyntheticChannels, chunk_size)) {
        std::fprintf(stderr, "Failed to start capture\n");
        return 1;
    }

    std::printf("=== Live 3ch Capture ===\n");
    std::printf("side: %.4f m | sample rate: %d Hz | device: %d\n",
                mic_distance, sample_rate, device_index);
    std::printf("Press Ctrl+C to stop.\n\n");

    g_stop_requested = 0;
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    auto last_status = std::chrono::steady_clock::now();
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (valid_count.load() == 0
            && now - last_status >= std::chrono::seconds(1)) {
            last_status = now;
            std::printf("\rWaiting for valid DOA... callbacks: %d, frames: %d, conf: %.3f    ",
                        callbacks.load(), ready_count.load(),
                        confidence_milli.load() / 1000.0f);
            std::fflush(stdout);
        }
    }

    capture.Stop();
    std::printf("\nStopped. callbacks: %d, frames: %d, valid: %d\n",
                callbacks.load(), ready_count.load(), valid_count.load());
    return 0;
}
#endif  // HAS_SPACEMIT_AUDIO

void PrintUsage(const char* prog) {
    std::printf("Usage:\n");
    std::printf("  %s -t [--geometry equilateral] [--mic-distance 0.063] [--angle DEG] [--sweep start:end:step]\n", prog);
    std::printf("  %s -f 3ch.wav [--mic-distance 0.063] [-v]\n", prog);
#ifdef HAS_SPACEMIT_AUDIO
    std::printf("  %s -l [--mic-distance 0.063] [-r rate] [-i device]\n", prog);
#endif
    std::printf("\nOptions:\n");
    std::printf("  -t, --test              Synthetic accuracy test\n");
    std::printf("  -f, --file WAV          Process 3-channel PCM16 WAV\n");
#ifdef HAS_SPACEMIT_AUDIO
    std::printf("  -l, --live              Live capture from 3-channel device\n");
#endif
    std::printf("  --geometry equilateral  Microphone geometry (default)\n");
    std::printf("  --mic-distance, -d M    Equilateral side length in meters (default: 0.063)\n");
    std::printf("  --angle DEG             Single synthetic angle\n");
    std::printf("  --sweep A:B:S           Synthetic sweep, inclusive\n");
    std::printf("  --noise-snr DB          Add Gaussian noise at the requested SNR\n");
    std::printf("  --azimuth-offset DEG    Array-frame to robot-frame offset\n");
    std::printf("  -r RATE                 Sample rate (default: 16000)\n");
    std::printf("  -i DEVICE               Capture device index (default: -1)\n");
    std::printf("  -v, --verbose           Print per-frame file/live results\n");
    std::printf("  --avg-seconds S         Sliding-window length for GetAverageAzimuth (s, default: 0 = unbounded since Reset)\n");
    std::printf("  --positions SPEC        Override mic positions, e.g. 'x0,y0;x1,y1;x2,y2' (meters); skips equilateral default\n");
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    const char* mode = nullptr;
    const char* wav_path = nullptr;
    std::string geometry = "equilateral";
    float mic_distance = kDefaultMicDistance;
    int sample_rate = 16000;
    int device_index = -1;
    bool verbose = false;
    bool have_angle = false;
    float angle = 90.0f;
    std::string sweep_spec;
    float azimuth_offset_deg = 0.0f;
    float noise_snr_db = std::numeric_limits<float>::infinity();
    float max_avg_seconds = 0.0f;
    std::vector<SpacemitAudio::MicrophonePosition> positions_override;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-t") == 0 || std::strcmp(argv[i], "--test") == 0) {
            mode = "test";
        } else if (std::strcmp(argv[i], "-f") == 0 || std::strcmp(argv[i], "--file") == 0) {
            mode = "file";
            if (i + 1 < argc) {
                wav_path = argv[++i];
            }
        } else if (std::strcmp(argv[i], "-l") == 0 || std::strcmp(argv[i], "--live") == 0) {
            mode = "live";
        } else if (std::strcmp(argv[i], "--geometry") == 0 && i + 1 < argc) {
            geometry = argv[++i];
        } else if ((std::strcmp(argv[i], "--mic-distance") == 0
                    || std::strcmp(argv[i], "-d") == 0) && i + 1 < argc) {
            mic_distance = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--angle") == 0 && i + 1 < argc) {
            angle = static_cast<float>(std::atof(argv[++i]));
            have_angle = true;
        } else if (std::strcmp(argv[i], "--sweep") == 0 && i + 1 < argc) {
            sweep_spec = argv[++i];
        } else if (std::strcmp(argv[i], "--noise-snr") == 0 && i + 1 < argc) {
            noise_snr_db = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--azimuth-offset") == 0 && i + 1 < argc) {
            azimuth_offset_deg = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            sample_rate = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            device_index = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "-v") == 0 || std::strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (std::strcmp(argv[i], "--avg-seconds") == 0 && i + 1 < argc) {
            max_avg_seconds = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--positions") == 0 && i + 1 < argc) {
            if (!ParsePositions(argv[++i], positions_override)) {
                std::fprintf(stderr, "Error: --positions must be N>=2 mics as 'x0,y0[,z0];x1,y1[,z1];...'\n");
                return 1;
            }
        }
    }

    if (!mode || (geometry != "equilateral" && positions_override.empty())) {
        PrintUsage(argv[0]);
        return 1;
    }

    if (std::strcmp(mode, "test") == 0) {
        std::vector<float> angles;
        if (!sweep_spec.empty()) {
            try {
                if (!ParseSweep(sweep_spec, angles)) {
                    std::fprintf(stderr, "Invalid --sweep value: %s\n", sweep_spec.c_str());
                    return 1;
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "Invalid --sweep value: %s (%s)\n",
                             sweep_spec.c_str(), e.what());
                return 1;
            }
        } else if (have_angle) {
            angles.push_back(NormalizeAngle360(angle));
        } else {
            ParseSweep("0:330:30", angles);
        }
        return RunSynthetic(mic_distance, sample_rate, angles,
                            azimuth_offset_deg, noise_snr_db,
                            max_avg_seconds, positions_override);
    }

    if (std::strcmp(mode, "file") == 0) {
        if (!wav_path) {
            std::fprintf(stderr, "Error: --file requires a WAV path\n\n");
            PrintUsage(argv[0]);
            return 1;
        }
        return RunFile(wav_path, mic_distance, verbose,
                       max_avg_seconds, positions_override,
                       azimuth_offset_deg);
    }

#ifdef HAS_SPACEMIT_AUDIO
    if (std::strcmp(mode, "live") == 0) {
        return RunLive(mic_distance, sample_rate, device_index,
                       verbose, azimuth_offset_deg,
                       max_avg_seconds, positions_override);
    }
#else
    if (std::strcmp(mode, "live") == 0) {
        std::fprintf(stderr, "Live mode is disabled: spacemit_audio not found at build time\n");
        return 1;
    }
#endif

    PrintUsage(argv[0]);
    return 1;
}
