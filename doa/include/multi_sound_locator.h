/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Multi-channel sound source localization API.
 */

#ifndef MULTI_SOUND_LOCATOR_H
#define MULTI_SOUND_LOCATOR_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace SpacemitAudio {

/// Microphone position in meters, centered in the array coordinate frame.
/// v1 uses x/y only for planar azimuth; z is reserved for future elevation.
struct MicrophonePosition {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/// Configuration for MultiSoundLocator.
struct MultiSoundLocatorConfig {
    int sample_rate = 16000;            ///< Sampling rate (Hz)
    int frame_size = 512;               ///< Samples per frame
    int avg_frames = 4;                 ///< Frames averaged before one result (one batch ≈ frame_size·avg_frames/sample_rate seconds; 128 ms at defaults)
    float max_avg_seconds = 0.0f;       ///< GetAverageAzimuth sliding-window length (s). 0 = unbounded since Reset (default, backward compat). >0 = keep only the most recent ceil(max_avg_seconds / batch_duration) batches in the running mean — useful for streaming / moving sources.
    int fft_size = 0;                   ///< FFT length; 0 = auto (2 * frame_size)
    int upsample_factor = 0;            ///< GCC upsample factor; 0 = auto (~4 us)
    float confidence_threshold = 0.1f;  ///< Minimum confidence for valid result
    float margin_threshold = 0.6f;      ///< Min ||k̂_unnorm|| (geometry consistency); silence/multi-source frames have margin < 0.5
    float sound_speed = 343.0f;         ///< Speed of sound (m/s)
    bool use_fftw_measure = false;      ///< Use FFTW_MEASURE for faster FFTs
    std::vector<MicrophonePosition> microphones;  ///< N >= 2; N=3 validated in v1
    float azimuth_offset_deg = 0.0f;    ///< Array-frame to robot-frame offset
    float max_frequency_hz = 0.0f;      ///< 0 = auto alias-safe c / (2 * d_max)
    float search_step_deg = 1.0f;       ///< Reserved for SRP-PHAT variants
};

/// Atomic snapshot of the latest multi-channel DOA result.
struct MultiSoundLocatorResult {
    float azimuth_deg = 0.0f;   ///< Robot-frame azimuth, [0, 360)
    float confidence = 0.0f;    ///< Mean pairwise GCC peak value
    float peak_score = 0.0f;    ///< MPCC-LSQ: same as confidence
    float score_margin = 0.0f;  ///< MPCC-LSQ: norm of the unnormalized wave vector
    int valid_pairs = 0;        ///< Number of pair peaks above confidence_threshold
    bool valid = false;         ///< confidence >= confidence_threshold AND score_margin >= margin_threshold
};

/// Multi-channel planar sound source locator.
///
/// v1 ships MPCC-LSQ: pairwise GCC-PHAT TDOAs are solved into a 2D wavefront
/// vector using a geometry-agnostic least-squares pseudo-inverse.
class MultiSoundLocator {
public:
    explicit MultiSoundLocator(const MultiSoundLocatorConfig& config);
    ~MultiSoundLocator();

    MultiSoundLocator(const MultiSoundLocator&) = delete;
    MultiSoundLocator& operator=(const MultiSoundLocator&) = delete;
    MultiSoundLocator(MultiSoundLocator&& other) noexcept;
    MultiSoundLocator& operator=(MultiSoundLocator&& other) noexcept;

    /// Allocate FFT plans and internal buffers. Must be called before Process().
    bool Initialize();

    /// Clear multi-frame accumulators and cached results.
    void Reset();

    /// Process interleaved float data [ch0, ch1, ..., chN-1, ch0, ...].
    /// Throws std::invalid_argument if n_channels does not match config.
    bool Process(const float* interleaved, size_t n_frames, int n_channels);

    /// Process separate float channel arrays.
    /// Throws std::invalid_argument if n_channels does not match config.
    bool Process(const float* const* channel_ptrs, size_t n_frames, int n_channels);

    /// Process interleaved PCM16 data.
    /// Throws std::invalid_argument if n_channels does not match config.
    bool Process(const int16_t* interleaved, size_t n_frames, int n_channels);

    MultiSoundLocatorResult GetResult() const;

    float GetAzimuth() const;
    float GetConfidence() const;
    bool IsValid() const;
    float GetAverageAzimuth() const;
    int GetResultCount() const;
    float GetTDOA(int i, int j) const;
    int GetMaxDelaySamplesPair(int i, int j) const;
    const MultiSoundLocatorConfig& GetConfig() const;

    static MultiSoundLocatorConfig CreateEquilateralTriangleConfig(
        float side_length_m = 0.063f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace SpacemitAudio

#endif  // MULTI_SOUND_LOCATOR_H
