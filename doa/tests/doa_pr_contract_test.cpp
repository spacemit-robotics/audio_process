/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "doa_service.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "ASSERTION FAILED: " << message << std::endl;
        std::exit(1);
    }
}

bool near(float actual, float expected, float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

std::vector<float> make_sine(size_t frames, int sample_rate, float frequency_hz) {
    std::vector<float> out(frames);
    for (size_t i = 0; i < frames; ++i) {
        const float phase = 2.0f * kPi * frequency_hz
            * static_cast<float>(i) / static_cast<float>(sample_rate);
        out[i] = std::sin(phase);
    }
    return out;
}

void verify_two_channel_broadside_contract() {
    SpacemitAudio::SoundLocatorConfig config;
    config.sample_rate = 16000;
    config.frame_size = 256;
    config.avg_frames = 1;
    config.confidence_threshold = 0.0f;
    config.upsample_factor = 4;
    config.use_fftw_measure = false;

    SpacemitAudio::SoundLocator locator(config);
    const auto signal = make_sine(config.frame_size, config.sample_rate, 1000.0f);

    require(!locator.Process(signal.data(), signal.data(), signal.size()),
            "Process before Initialize must return false");
    require(locator.Initialize(), "SoundLocator must initialize for synthetic audio");
    require(locator.GetMaxDelaySamples() > 0, "SoundLocator must compute physical max delay");
    require(locator.Process(signal.data(), signal.data(), signal.size()),
            "identical channels must produce a ready DOA result");
    require(near(locator.GetTDOA(), 0.0f, 1.0f / config.sample_rate),
            "identical channels must produce near-zero TDOA");
    require(near(locator.GetDOA(), 90.0f, 5.0f),
            "identical channels must produce broadside DOA near 90 degrees");
    require(locator.GetConfidence() > 0.0f, "synthetic sine must produce positive confidence");
    require(locator.IsValid(), "zero-threshold synthetic result must be valid");
    require(locator.GetResultCount() == 1, "valid synthetic result must update result count");

    locator.Reset();
    require(locator.GetResultCount() == 0, "Reset must clear result count");
    require(!locator.IsValid(), "Reset must clear validity");
}

void verify_multi_channel_geometry_contract() {
    auto config = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(0.063f);
    config.sample_rate = 16000;
    config.frame_size = 256;
    config.avg_frames = 1;
    config.confidence_threshold = 0.0f;
    config.margin_threshold = 0.0f;
    config.quality_threshold = 0.0f;
    config.closure_threshold_fraction = 1.0f;
    config.upsample_factor = 4;
    config.use_fftw_measure = false;

    require(config.microphones.size() == 3, "equilateral config must create three microphones");
    require(config.microphones[0].x > 0.0f, "mic0 must be on +x axis");

    SpacemitAudio::MultiSoundLocator locator(config);
    require(locator.Initialize(), "MultiSoundLocator must initialize for equilateral config");
    require(locator.GetPairCount() == 3, "three microphones must produce three pairs");
    require(locator.GetMaxDelaySamplesPair(0, 1) > 0, "pair max delay must be positive");

    const auto signal = make_sine(config.frame_size, config.sample_rate, 1000.0f);
    std::vector<float> interleaved;
    interleaved.reserve(signal.size() * 3);
    for (float sample : signal) {
        interleaved.push_back(sample);
        interleaved.push_back(sample);
        interleaved.push_back(sample);
    }

    require(locator.Process(interleaved.data(), signal.size(), 3),
            "identical 3ch input must produce a ready result");
    require(near(locator.GetClosureResidual(), 0.0f, 1.0f / config.sample_rate),
            "identical 3ch input must have near-zero closure residual");

    float pair_confidences[3] = {};
    locator.GetPairConfidences(pair_confidences, 3);
    require(pair_confidences[0] >= 0.0f && pair_confidences[1] >= 0.0f
            && pair_confidences[2] >= 0.0f,
            "pair confidences must be populated");
}

void verify_invalid_input_error_path() {
    SpacemitAudio::SoundLocatorConfig sound_config;
    sound_config.sample_rate = 16000;
    sound_config.frame_size = 256;
    sound_config.avg_frames = 1;
    SpacemitAudio::SoundLocator sound_locator(sound_config);
    const auto signal = make_sine(sound_config.frame_size, sound_config.sample_rate, 1000.0f);
    require(!sound_locator.Process(signal.data(), signal.data(), signal.size()),
            "2ch Process before Initialize must return false");
    require(!sound_locator.Process(nullptr, signal.data(), signal.size()),
            "2ch Process with null channel must return false");

    auto bad_config = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(0.063f);
    bad_config.sample_rate = 0;
    SpacemitAudio::MultiSoundLocator bad_locator(bad_config);
    require(!bad_locator.Initialize(), "invalid sample rate must fail initialization");

    auto config = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(0.063f);
    config.sample_rate = 16000;
    config.frame_size = 256;
    config.avg_frames = 1;
    SpacemitAudio::MultiSoundLocator locator(config);
    require(locator.Initialize(), "valid MultiSoundLocator config must initialize");

    bool wrong_channels_threw = false;
    try {
        (void)locator.Process(signal.data(), signal.size() / 3, 2);
    } catch (const std::invalid_argument& exc) {
        wrong_channels_threw =
            std::string(exc.what()).find("channels") != std::string::npos;
    }
    require(wrong_channels_threw, "wrong channel count must throw invalid_argument");

    bool bad_pair_count_threw = false;
    try {
        float out[2] = {};
        locator.GetPairConfidences(out, 2);
    } catch (const std::out_of_range& exc) {
        bad_pair_count_threw =
            std::string(exc.what()).find("pair count") != std::string::npos;
    }
    require(bad_pair_count_threw, "bad pair confidence buffer length must throw out_of_range");
}

}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "expected one test mode argument");
    const std::string mode = argv[1];

    if (mode == "--synthetic-gcc-contract") {
        verify_two_channel_broadside_contract();
        verify_multi_channel_geometry_contract();
    } else if (mode == "--invalid-input-error-path") {
        verify_invalid_input_error_path();
    } else {
        std::cerr << "Unknown mode: " << mode << std::endl;
        return 2;
    }

    std::cout << "PASS " << mode << std::endl;
    return 0;
}
