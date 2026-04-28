/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * pybind11 bindings for SoundLocator
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "doa_service.h"

namespace py = pybind11;

using SpacemitAudio::SoundLocator;
using SpacemitAudio::SoundLocatorConfig;

// Wrapper to accept numpy arrays
class PySoundLocator {
public:
    explicit PySoundLocator(const SoundLocatorConfig& config = {})
        : loc_(config) {}

    bool initialize() { return loc_.Initialize(); }
    void reset() { loc_.Reset(); }

    // Interleaved float32 numpy array [L, R, L, R, ...]
    bool process_float(py::array_t<float> interleaved) {
        auto buf = interleaved.request();
        if (buf.ndim != 1) {
            throw std::runtime_error("Expected 1-D array for interleaved data");
        }
        size_t total = static_cast<size_t>(buf.shape[0]);
        if (total % 2 != 0) {
            throw std::runtime_error("Interleaved array length must be even");
        }
        const auto* ptr = static_cast<const float*>(buf.ptr);
        py::gil_scoped_release release;
        return loc_.Process(ptr, total / 2);
    }

    // Interleaved int16 numpy array
    bool process_int16(py::array_t<int16_t> interleaved) {
        auto buf = interleaved.request();
        if (buf.ndim != 1) {
            throw std::runtime_error("Expected 1-D array for interleaved data");
        }
        size_t total = static_cast<size_t>(buf.shape[0]);
        if (total % 2 != 0) {
            throw std::runtime_error("Interleaved array length must be even");
        }
        const auto* ptr = static_cast<const int16_t*>(buf.ptr);
        py::gil_scoped_release release;
        return loc_.Process(ptr, total / 2);
    }

    // Two separate float32 numpy arrays
    bool process_separate(py::array_t<float> ch0, py::array_t<float> ch1) {
        auto b0 = ch0.request();
        auto b1 = ch1.request();
        if (b0.ndim != 1 || b1.ndim != 1) {
            throw std::runtime_error("Expected 1-D arrays");
        }
        if (b0.shape[0] != b1.shape[0]) {
            throw std::runtime_error("ch0 and ch1 must have same length");
        }
        size_t n = static_cast<size_t>(b0.shape[0]);
        const auto* p0 = static_cast<const float*>(b0.ptr);
        const auto* p1 = static_cast<const float*>(b1.ptr);
        py::gil_scoped_release release;
        return loc_.Process(p0, p1, n);
    }

    float get_tdoa() const { return loc_.GetTDOA(); }
    float get_doa() const { return loc_.GetDOA(); }
    float get_confidence() const { return loc_.GetConfidence(); }
    bool is_valid() const { return loc_.IsValid(); }
    float get_average_doa() const { return loc_.GetAverageDOA(); }
    int get_result_count() const { return loc_.GetResultCount(); }
    SoundLocatorConfig get_config() const { return loc_.GetConfig(); }
    int get_max_delay_samples() const { return loc_.GetMaxDelaySamples(); }

    PySoundLocator& enter() { return *this; }
    void exit(py::object, py::object, py::object) { reset(); }

private:
    SoundLocator loc_;
};

PYBIND11_MODULE(_spacemit_audio_process, m) {
    m.doc() = "Sound source localization (GCC-PHAT)";

    py::class_<SoundLocatorConfig>(m, "SoundLocatorConfig")
        .def(py::init<>())
        .def_readwrite("sample_rate", &SoundLocatorConfig::sample_rate)
        .def_readwrite("mic_distance", &SoundLocatorConfig::mic_distance)
        .def_readwrite("sound_speed", &SoundLocatorConfig::sound_speed)
        .def_readwrite("fft_size", &SoundLocatorConfig::fft_size)
        .def_readwrite("frame_size", &SoundLocatorConfig::frame_size)
        .def_readwrite("avg_frames", &SoundLocatorConfig::avg_frames)
        .def_readwrite("confidence_threshold",
            &SoundLocatorConfig::confidence_threshold)
        .def_readwrite("upsample_factor",
            &SoundLocatorConfig::upsample_factor)
        .def_readwrite("use_fftw_measure",
            &SoundLocatorConfig::use_fftw_measure)
        .def("__repr__", [](const SoundLocatorConfig& c) {
            return "<SoundLocatorConfig sr=" + std::to_string(c.sample_rate)
                + " mic_dist=" + std::to_string(c.mic_distance)
                + " frame=" + std::to_string(c.frame_size)
                + " avg=" + std::to_string(c.avg_frames) + ">";
        });

    py::class_<PySoundLocator>(m, "SoundLocator")
        .def(py::init<const SoundLocatorConfig&>(),
            py::arg("config") = SoundLocatorConfig{})
        .def("initialize", &PySoundLocator::initialize)
        .def("reset", &PySoundLocator::reset)
        .def("process", &PySoundLocator::process_float,
            py::arg("interleaved"),
            "Process interleaved float32 stereo data")
        .def("process_int16", &PySoundLocator::process_int16,
            py::arg("interleaved"),
            "Process interleaved int16 stereo data")
        .def("process_separate", &PySoundLocator::process_separate,
            py::arg("ch0"), py::arg("ch1"),
            "Process two separate float32 channel arrays")
        .def_property_readonly("tdoa", &PySoundLocator::get_tdoa)
        .def_property_readonly("doa", &PySoundLocator::get_doa)
        .def_property_readonly("confidence", &PySoundLocator::get_confidence)
        .def_property_readonly("is_valid", &PySoundLocator::is_valid)
        .def_property_readonly("average_doa", &PySoundLocator::get_average_doa)
        .def_property_readonly("result_count", &PySoundLocator::get_result_count)
        .def_property_readonly("config", &PySoundLocator::get_config)
        .def_property_readonly("max_delay_samples",
            &PySoundLocator::get_max_delay_samples)
        .def("__enter__", &PySoundLocator::enter,
            py::return_value_policy::reference)
        .def("__exit__", &PySoundLocator::exit);
}
