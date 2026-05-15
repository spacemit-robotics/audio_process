/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Multi-channel planar sound source localization using pairwise GCC-PHAT
 * and MPCC-LSQ fusion.
 */

#include "multi_sound_locator.h"

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include "gcc_phat_pair.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SpacemitAudio {
namespace {

constexpr float kEpsilon = 1e-10f;

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

float Distance2D(const MicrophonePosition& a, const MicrophonePosition& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

struct MultiSoundLocator::Impl {
    struct PairInfo {
        int i = 0;
        int j = 0;
        float row_x = 0.0f;  // p_j.x - p_i.x, matching GCC sign t_i - t_j
        float row_y = 0.0f;  // p_j.y - p_i.y
        float distance_m = 0.0f;
    };

    MultiSoundLocatorConfig config;
    int n_channels = 0;
    int upsample_factor = 1;
    int padded_size = 0;
    int spectrum_size = 0;
    int upsampled_size = 0;
    int up_spectrum_size = 0;

    std::vector<float*> fft_in;
    std::vector<fftwf_complex*> fft_out;
    std::vector<fftwf_plan> fft_plans;
    std::vector<float> hann_window;

    std::vector<PairInfo> pairs;
    std::vector<internal::GccPhatPair> gcc_pairs;
    std::vector<float> latest_tdoa;

    // Cholesky factor of A^T A = L * L^T for the 2D LSQ solve.
    float chol_l00 = 0.0f;
    float chol_l10 = 0.0f;
    float chol_l11 = 0.0f;

    MultiSoundLocatorResult result;

    // Sliding-window accumulator for GetAverageAzimuth. If
    // config.max_avg_seconds <= 0, the deque is unbounded (matches v1
    // behavior). Otherwise it is capped at max_avg_batches and the running
    // sum is decremented on eviction so GetAverageAzimuth stays O(1).
    struct BatchEntry { float weighted_x; float weighted_y; };
    std::deque<BatchEntry> avg_history;
    int   max_avg_batches = 0;   // 0 = unbounded
    float average_x = 0.0f;
    float average_y = 0.0f;
    int   result_count = 0;

    std::vector<std::vector<float>> planar_buffers;
    std::vector<float> int16_float_buffer;
    std::vector<const float*> channel_view;

    bool initialized = false;

    void DestroyFFT();
    bool BuildGeometry();
    bool SolveNormal(float rhs_x, float rhs_y, float& out_x, float& out_y) const;
    void ValidateChannelCount(int n) const;
    bool ProcessPlanar(const float* const* channel_ptrs, size_t n_frames);
    bool ProcessChunk(const float* const* channel_ptrs, size_t chunk);
    bool SolveReadyPairs();
    int FindPairIndex(int i, int j) const;
};

void MultiSoundLocator::Impl::DestroyFFT() {
    for (auto plan : fft_plans) {
        if (plan) {
            fftwf_destroy_plan(plan);
        }
    }
    fft_plans.clear();

    for (auto* ptr : fft_in) {
        if (ptr) {
            fftwf_free(ptr);
        }
    }
    fft_in.clear();

    for (auto* ptr : fft_out) {
        if (ptr) {
            fftwf_free(ptr);
        }
    }
    fft_out.clear();

    gcc_pairs.clear();
    pairs.clear();
    latest_tdoa.clear();
    hann_window.clear();
    initialized = false;
}

bool MultiSoundLocator::Impl::BuildGeometry() {
    pairs.clear();

    float max_pair_distance = 0.0f;
    for (int i = 0; i < n_channels; ++i) {
        for (int j = i + 1; j < n_channels; ++j) {
            PairInfo pair;
            pair.i = i;
            pair.j = j;
            pair.row_x = config.microphones[j].x - config.microphones[i].x;
            pair.row_y = config.microphones[j].y - config.microphones[i].y;
            pair.distance_m = Distance2D(config.microphones[i],
                                         config.microphones[j]);
            max_pair_distance = std::max(max_pair_distance, pair.distance_m);
            pairs.push_back(pair);
        }
    }

    if (pairs.empty() || max_pair_distance <= 0.0f) {
        return false;
    }

    if (config.max_frequency_hz <= 0.0f) {
        config.max_frequency_hz = config.sound_speed / (2.0f * max_pair_distance);
    }

    float ata00 = 0.0f;
    float ata01 = 0.0f;
    float ata11 = 0.0f;
    for (const auto& pair : pairs) {
        ata00 += pair.row_x * pair.row_x;
        ata01 += pair.row_x * pair.row_y;
        ata11 += pair.row_y * pair.row_y;
    }

    if (ata00 <= kEpsilon) {
        return false;
    }
    chol_l00 = std::sqrt(ata00);
    chol_l10 = ata01 / chol_l00;
    const float diag = ata11 - chol_l10 * chol_l10;
    if (diag <= kEpsilon) {
        return false;
    }
    chol_l11 = std::sqrt(diag);
    return true;
}

bool MultiSoundLocator::Impl::SolveNormal(
    float rhs_x, float rhs_y, float& out_x, float& out_y) const {
    if (chol_l00 <= 0.0f || chol_l11 <= 0.0f) {
        return false;
    }

    const float y0 = rhs_x / chol_l00;
    const float y1 = (rhs_y - chol_l10 * y0) / chol_l11;
    out_y = y1 / chol_l11;
    out_x = (y0 - chol_l10 * out_y) / chol_l00;
    return true;
}

void MultiSoundLocator::Impl::ValidateChannelCount(int n) const {
    if (n != n_channels) {
        throw std::invalid_argument(
            "MultiSoundLocator::Process expected " + std::to_string(n_channels)
            + " channels, got " + std::to_string(n));
    }
}

bool MultiSoundLocator::Impl::ProcessPlanar(
    const float* const* channel_ptrs, size_t n_frames) {
    if (!initialized || !channel_ptrs || n_frames == 0) {
        return false;
    }

    for (int ch = 0; ch < n_channels; ++ch) {
        if (!channel_ptrs[ch]) {
            return false;
        }
    }

    const size_t frame = static_cast<size_t>(config.frame_size);
    size_t offset = 0;
    bool result_ready = false;

    while (offset < n_frames) {
        const size_t chunk = std::min(frame, n_frames - offset);
        channel_view.resize(n_channels);
        for (int ch = 0; ch < n_channels; ++ch) {
            channel_view[ch] = channel_ptrs[ch] + offset;
        }
        if (ProcessChunk(channel_view.data(), chunk)) {
            result_ready = true;
        }
        offset += chunk;
    }

    return result_ready;
}

bool MultiSoundLocator::Impl::ProcessChunk(
    const float* const* channel_ptrs, size_t chunk) {
    const size_t copy_len = std::min(
        chunk, static_cast<size_t>(config.frame_size));

    for (int ch = 0; ch < n_channels; ++ch) {
        std::memset(fft_in[ch], 0, padded_size * sizeof(float));
        for (size_t i = 0; i < copy_len; ++i) {
            fft_in[ch][i] = channel_ptrs[ch][i] * hann_window[i];
        }
        fftwf_execute(fft_plans[ch]);
    }

    for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
        const auto& pair = pairs[pair_idx];
        if (!gcc_pairs[pair_idx].Accumulate(fft_out[pair.i], fft_out[pair.j])) {
            return false;
        }
    }

    const bool all_ready = std::all_of(
        gcc_pairs.begin(), gcc_pairs.end(),
        [](const internal::GccPhatPair& pair) { return pair.IsReady(); });
    if (!all_ready) {
        return false;
    }

    return SolveReadyPairs();
}

bool MultiSoundLocator::Impl::SolveReadyPairs() {
    float rhs_x = 0.0f;
    float rhs_y = 0.0f;
    float confidence_sum = 0.0f;
    int valid_pairs = 0;

    for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
        const auto pair_result = gcc_pairs[pair_idx].IfftAndPeak();
        if (!pair_result.ready) {
            return false;
        }

        latest_tdoa[pair_idx] = pair_result.tdoa_seconds;
        const float distance_delta = config.sound_speed * pair_result.tdoa_seconds;
        rhs_x += pairs[pair_idx].row_x * distance_delta;
        rhs_y += pairs[pair_idx].row_y * distance_delta;
        confidence_sum += pair_result.peak_value;
        if (pair_result.peak_value >= config.confidence_threshold) {
            ++valid_pairs;
        }
    }

    float wave_x = 0.0f;
    float wave_y = 0.0f;
    if (!SolveNormal(rhs_x, rhs_y, wave_x, wave_y)) {
        return false;
    }

    const float norm = std::sqrt(wave_x * wave_x + wave_y * wave_y);
    const float azimuth_rad = std::atan2(wave_y, wave_x);
    const float azimuth_deg = NormalizeAngle360(
        azimuth_rad * 180.0f / static_cast<float>(M_PI)
        + config.azimuth_offset_deg);
    const float confidence = confidence_sum / static_cast<float>(pairs.size());

    result.azimuth_deg = azimuth_deg;
    result.confidence = confidence;
    result.peak_score = confidence;
    result.score_margin = norm;
    result.valid_pairs = valid_pairs;
    // valid requires BOTH a coherent cross-spectrum (confidence) AND mutually
    // consistent pairwise TDOAs (margin = ||k̂_unnorm|| close to 1). Silence /
    // multi-source / strong-reverb frames typically have margin < 0.5 and
    // would otherwise pollute the vector-space GetAverageAzimuth average.
    result.valid = confidence >= config.confidence_threshold
                && norm >= config.margin_threshold;

    if (result.valid && confidence > 0.0f) {
        const float robot_rad = azimuth_deg * static_cast<float>(M_PI) / 180.0f;
        BatchEntry entry;
        entry.weighted_x = confidence * std::cos(robot_rad);
        entry.weighted_y = confidence * std::sin(robot_rad);
        average_x += entry.weighted_x;
        average_y += entry.weighted_y;
        avg_history.push_back(entry);
        ++result_count;
        // Sliding-window eviction: drop oldest batches until the deque fits
        // the configured max_avg_batches. Running sum decrements in lockstep
        // so GetAverageAzimuth stays O(1).
        if (max_avg_batches > 0) {
            while (static_cast<int>(avg_history.size()) > max_avg_batches) {
                const auto& front = avg_history.front();
                average_x -= front.weighted_x;
                average_y -= front.weighted_y;
                --result_count;
                avg_history.pop_front();
            }
        }
    }

    return true;
}

int MultiSoundLocator::Impl::FindPairIndex(int i, int j) const {
    if (i < 0 || j < 0 || i >= n_channels || j >= n_channels || i == j) {
        throw std::out_of_range("microphone pair index out of range");
    }

    const int lo = std::min(i, j);
    const int hi = std::max(i, j);
    for (size_t idx = 0; idx < pairs.size(); ++idx) {
        if (pairs[idx].i == lo && pairs[idx].j == hi) {
            return static_cast<int>(idx);
        }
    }

    throw std::out_of_range("microphone pair index out of range");
}

MultiSoundLocator::MultiSoundLocator(const MultiSoundLocatorConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
}

MultiSoundLocator::~MultiSoundLocator() {
    if (impl_) {
        impl_->DestroyFFT();
    }
}

MultiSoundLocator::MultiSoundLocator(MultiSoundLocator&& other) noexcept = default;
MultiSoundLocator& MultiSoundLocator::operator=(MultiSoundLocator&& other) noexcept = default;

bool MultiSoundLocator::Initialize() {
    auto& d = *impl_;
    d.DestroyFFT();

    d.n_channels = static_cast<int>(d.config.microphones.size());
    if (d.n_channels < 2 || d.config.sample_rate <= 0
        || d.config.frame_size <= 0 || d.config.avg_frames <= 0
        || d.config.sound_speed <= 0.0f) {
        return false;
    }

    d.padded_size = d.config.fft_size > 0
                        ? d.config.fft_size
                        : 2 * d.config.frame_size;
    if (d.padded_size < d.config.frame_size) {
        d.padded_size = d.config.frame_size;
    }
    if (d.padded_size % 2 != 0) {
        ++d.padded_size;
    }
    d.spectrum_size = d.padded_size / 2 + 1;

    if (d.config.upsample_factor > 0) {
        d.upsample_factor = d.config.upsample_factor;
    } else {
        d.upsample_factor = std::max(1,
            static_cast<int>(std::ceil(
                1.0 / (4e-6 * static_cast<double>(d.config.sample_rate)))));
    }
    d.upsampled_size = d.padded_size * d.upsample_factor;
    d.up_spectrum_size = d.upsampled_size / 2 + 1;

    if (!d.BuildGeometry()) {
        d.DestroyFFT();
        return false;
    }

    d.fft_in.assign(d.n_channels, nullptr);
    d.fft_out.assign(d.n_channels, nullptr);
    d.fft_plans.assign(d.n_channels, nullptr);

    const unsigned fftw_flags = d.config.use_fftw_measure
                                    ? FFTW_MEASURE : FFTW_ESTIMATE;
    for (int ch = 0; ch < d.n_channels; ++ch) {
        d.fft_in[ch] = fftwf_alloc_real(d.padded_size);
        d.fft_out[ch] = fftwf_alloc_complex(d.spectrum_size);
        if (!d.fft_in[ch] || !d.fft_out[ch]) {
            d.DestroyFFT();
            return false;
        }
        d.fft_plans[ch] = fftwf_plan_dft_r2c_1d(
            d.padded_size, d.fft_in[ch], d.fft_out[ch], fftw_flags);
        if (!d.fft_plans[ch]) {
            d.DestroyFFT();
            return false;
        }
    }

    d.hann_window.resize(d.config.frame_size);
    if (d.config.frame_size == 1) {
        d.hann_window[0] = 1.0f;
    } else {
        for (int i = 0; i < d.config.frame_size; ++i) {
            d.hann_window[i] = 0.5f * (1.0f - std::cos(
                2.0f * static_cast<float>(M_PI) * i
                / static_cast<float>(d.config.frame_size - 1)));
        }
    }

    d.gcc_pairs.clear();
    d.gcc_pairs.reserve(d.pairs.size());
    for (const auto& pair : d.pairs) {
        internal::GccPhatPairConfig pair_config;
        pair_config.sample_rate = d.config.sample_rate;
        pair_config.avg_frames = d.config.avg_frames;
        pair_config.padded_size = d.padded_size;
        pair_config.spectrum_size = d.spectrum_size;
        pair_config.upsample_factor = d.upsample_factor;
        pair_config.upsampled_size = d.upsampled_size;
        pair_config.up_spectrum_size = d.up_spectrum_size;
        pair_config.sound_speed = d.config.sound_speed;
        pair_config.pair_distance_m = pair.distance_m;
        pair_config.max_frequency_hz = d.config.max_frequency_hz;
        pair_config.use_fftw_measure = d.config.use_fftw_measure;

        internal::GccPhatPair gcc_pair;
        if (!gcc_pair.Initialize(pair_config)) {
            d.DestroyFFT();
            return false;
        }
        d.gcc_pairs.push_back(std::move(gcc_pair));
    }

    d.latest_tdoa.assign(d.pairs.size(), 0.0f);
    d.planar_buffers.assign(d.n_channels, {});
    d.channel_view.assign(d.n_channels, nullptr);
    d.result = MultiSoundLocatorResult{};
    d.avg_history.clear();
    d.average_x = 0.0f;
    d.average_y = 0.0f;
    d.result_count = 0;

    // Compute sliding-window cap for GetAverageAzimuth from max_avg_seconds.
    // 0 (default) = unbounded since Reset (v1 backward-compat).
    // >0 = ceil(seconds / batch_duration_seconds), min 1.
    d.max_avg_batches = 0;
    if (d.config.max_avg_seconds > 0.0f && d.config.sample_rate > 0
        && d.config.frame_size > 0 && d.config.avg_frames > 0) {
        const double batch_seconds =
            static_cast<double>(d.config.avg_frames) *
            static_cast<double>(d.config.frame_size) /
            static_cast<double>(d.config.sample_rate);
        if (batch_seconds > 0.0) {
            const double batches =
                static_cast<double>(d.config.max_avg_seconds) / batch_seconds;
            d.max_avg_batches = std::max(1, static_cast<int>(std::ceil(batches)));
        }
    }

    d.initialized = true;
    return true;
}

void MultiSoundLocator::Reset() {
    if (!impl_ || !impl_->initialized) {
        return;
    }

    for (auto& pair : impl_->gcc_pairs) {
        pair.Reset();
    }
    std::fill(impl_->latest_tdoa.begin(), impl_->latest_tdoa.end(), 0.0f);
    impl_->result = MultiSoundLocatorResult{};
    impl_->avg_history.clear();
    impl_->average_x = 0.0f;
    impl_->average_y = 0.0f;
    impl_->result_count = 0;
}

bool MultiSoundLocator::Process(
    const float* interleaved, size_t n_frames, int n_channels) {
    if (!impl_ || !impl_->initialized || !interleaved || n_frames == 0) {
        return false;
    }
    impl_->ValidateChannelCount(n_channels);

    impl_->planar_buffers.assign(impl_->n_channels,
                                 std::vector<float>(n_frames));
    for (size_t frame = 0; frame < n_frames; ++frame) {
        for (int ch = 0; ch < impl_->n_channels; ++ch) {
            impl_->planar_buffers[ch][frame] =
                interleaved[frame * impl_->n_channels + ch];
        }
    }

    impl_->channel_view.resize(impl_->n_channels);
    for (int ch = 0; ch < impl_->n_channels; ++ch) {
        impl_->channel_view[ch] = impl_->planar_buffers[ch].data();
    }
    return impl_->ProcessPlanar(impl_->channel_view.data(), n_frames);
}

bool MultiSoundLocator::Process(
    const float* const* channel_ptrs, size_t n_frames, int n_channels) {
    if (!impl_ || !impl_->initialized || n_frames == 0) {
        return false;
    }
    impl_->ValidateChannelCount(n_channels);
    return impl_->ProcessPlanar(channel_ptrs, n_frames);
}

bool MultiSoundLocator::Process(
    const int16_t* interleaved, size_t n_frames, int n_channels) {
    if (!impl_ || !impl_->initialized || !interleaved || n_frames == 0) {
        return false;
    }
    impl_->ValidateChannelCount(n_channels);

    constexpr float kScale = 1.0f / 32768.0f;
    const size_t total = n_frames * static_cast<size_t>(impl_->n_channels);
    impl_->int16_float_buffer.resize(total);
    for (size_t i = 0; i < total; ++i) {
        impl_->int16_float_buffer[i] = static_cast<float>(interleaved[i]) * kScale;
    }
    return Process(impl_->int16_float_buffer.data(), n_frames, n_channels);
}

MultiSoundLocatorResult MultiSoundLocator::GetResult() const {
    return impl_->result;
}

float MultiSoundLocator::GetAzimuth() const {
    return impl_->result.azimuth_deg;
}

float MultiSoundLocator::GetConfidence() const {
    return impl_->result.confidence;
}

bool MultiSoundLocator::IsValid() const {
    return impl_->result.valid;
}

float MultiSoundLocator::GetAverageAzimuth() const {
    if (impl_->result_count <= 0) {
        return impl_->result.azimuth_deg;
    }
    if (std::fabs(impl_->average_x) <= kEpsilon
        && std::fabs(impl_->average_y) <= kEpsilon) {
        return impl_->result.azimuth_deg;
    }
    return NormalizeAngle360(
        std::atan2(impl_->average_y, impl_->average_x)
        * 180.0f / static_cast<float>(M_PI));
}

int MultiSoundLocator::GetResultCount() const {
    return impl_->result_count;
}

float MultiSoundLocator::GetTDOA(int i, int j) const {
    const int idx = impl_->FindPairIndex(i, j);
    const float value = impl_->latest_tdoa[idx];
    return i < j ? value : -value;
}

int MultiSoundLocator::GetMaxDelaySamplesPair(int i, int j) const {
    const int idx = impl_->FindPairIndex(i, j);
    return impl_->gcc_pairs[idx].GetMaxDelaySamples();
}

const MultiSoundLocatorConfig& MultiSoundLocator::GetConfig() const {
    return impl_->config;
}

MultiSoundLocatorConfig MultiSoundLocator::CreateEquilateralTriangleConfig(
    float side_length_m) {
    MultiSoundLocatorConfig config;
    const float sqrt3 = std::sqrt(3.0f);
    config.microphones = {
        {side_length_m / sqrt3, 0.0f, 0.0f},
        {-side_length_m / (2.0f * sqrt3), side_length_m / 2.0f, 0.0f},
        {-side_length_m / (2.0f * sqrt3), -side_length_m / 2.0f, 0.0f},
    };
    config.azimuth_offset_deg = 0.0f;
    return config;
}

}  // namespace SpacemitAudio
