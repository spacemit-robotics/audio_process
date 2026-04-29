/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sound Source Localization (SSL) — dual-channel GCC-PHAT algorithm
 */

#ifndef DOA_SERVICE_H
#define DOA_SERVICE_H

#include <cstddef>
#include <cstdint>
#include <memory>

namespace SpacemitAudio {

/// Configuration for SoundLocator.
/// All fields have sensible defaults; users MUST set `mic_distance`
/// to match their hardware.
struct SoundLocatorConfig {
    int sample_rate = 16000;            ///< Sampling rate (Hz)
    float mic_distance = 0.058f;        ///< Microphone spacing (meters)
    float sound_speed = 343.0f;         ///< Speed of sound (m/s)
    int fft_size = 0;                   ///< FFT length; 0 = auto (2 * frame_size)
    int frame_size = 512;               ///< Samples per frame (512 @ 16 kHz ≈ 32 ms)
    int avg_frames = 4;                 ///< Number of frames to average before output
    float confidence_threshold = 0.1f;  ///< Minimum confidence to consider result valid
    int upsample_factor = 0;            ///< GCC upsample factor; 0 = auto (~4μs resolution)
    bool use_fftw_measure = false;      ///< Use FFTW_MEASURE for faster FFTs (slower init)
};

/// Dual-channel sound source locator using GCC-PHAT.
///
/// Usage:
/// @code
///   SoundLocatorConfig cfg;
///   cfg.mic_distance = 0.10f;          // set to your mic spacing
///   SoundLocator loc(cfg);
///   loc.Initialize();
///
///   // feed stereo PCM16 frames (from AudioCapture callback)
///   loc.Process(pcm16_interleaved, num_frames);
///
///   if (loc.IsValid()) {
///       float angle = loc.GetDOA();    // degrees, [0, 180]
///   }
/// @endcode
class SoundLocator {
public:
    explicit SoundLocator(const SoundLocatorConfig& config = {});
    ~SoundLocator();

    SoundLocator(const SoundLocator&) = delete;
    SoundLocator& operator=(const SoundLocator&) = delete;
    SoundLocator(SoundLocator&& other) noexcept;
    SoundLocator& operator=(SoundLocator&& other) noexcept;

    /// Allocate FFT plans and internal buffers.  Must be called once
    /// before the first Process().
    /// @return true on success.
    bool Initialize();

    /// Clear the multi-frame accumulator and cached results.
    void Reset();

    // ----- Process methods ------------------------------------------------

    /// Process interleaved stereo float data [L, R, L, R, ...].
    /// @param interleaved  pointer to interleaved samples
    /// @param num_frames   number of sample frames (each frame = 2 floats)
    /// @return true if a DOA result is ready (enough frames accumulated).
    bool Process(const float* interleaved, size_t num_frames);

    /// Process two separate float channels.
    bool Process(const float* ch0, const float* ch1, size_t num_frames);

    /// Process interleaved stereo PCM16 data (compatible with AudioCapture).
    bool Process(const int16_t* interleaved, size_t num_frames);

    // ----- Result accessors -----------------------------------------------

    /// Estimated time-difference-of-arrival (seconds).
    float GetTDOA() const;

    /// Estimated direction-of-arrival (degrees), range [0, 180].
    /// 90° = broadside (perpendicular), 0° = endfire (ch1 side), 180° = endfire (ch0 side).
    float GetDOA() const;

    /// Peak value of the normalised GCC function, range [0, 1].
    float GetConfidence() const;

    /// True when GetConfidence() >= confidence_threshold.
    bool IsValid() const;

    /// Dominant confidence-weighted DOA across all processed batches since
    /// last Reset(). Aggregation is done in TDOA space so near-endfire
    /// results are not pulled toward 90° by a few opposite-sign outliers.
    /// Returns 90.0f (broadside) if no valid results yet.
    float GetAverageDOA() const;

    /// Number of valid result batches accumulated since last Reset().
    int GetResultCount() const;

    // ----- Configuration query --------------------------------------------

    const SoundLocatorConfig& GetConfig() const;

    /// Maximum delay in samples that is physically possible given
    /// mic_distance and sound_speed.
    int GetMaxDelaySamples() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace SpacemitAudio

#endif  // DOA_SERVICE_H
