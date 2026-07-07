# DOA API

声源定位（Direction of Arrival, DOA）组件，提供双声道 GCC-PHAT 和 3+ 声道平面 MPCC-LSQ 声源方位估计，支持 C++ 和 Python。

## 功能特性

- **双声道定位**: `SoundLocator`，输出 DOA 角度 `[0°, 180°]`，90° 为麦克风连线正侧向。
- **多声道定位**: `MultiSoundLocator`，输出机器人坐标系方位角 `[0°, 360°)`。
- **多种输入格式**: float32 交错、float32 分离通道、PCM16 交错。
- **可观测性指标**: TDOA、confidence、quality、closure residual、pair confidences、平均方位稳定性。
- **实时音频友好**: `Process()` 可重复喂入短帧；live 场景可设置滑动平均窗口避免无界历史累积。

---

## 坐标与通道约定

- `SoundLocator`（2ch）:
  - 输出 `DOA ∈ [0°, 180°]`
  - `90°` = broadside，垂直于双麦连线
  - `0°` = 靠近 ch1 的 endfire
  - `180°` = 靠近 ch0 的 endfire
  - 2ch 无前后区分；需要完整 360° 方位时使用 3+ch `MultiSoundLocator`

- `MultiSoundLocator`（3+ch）:
  - 输出 `azimuth_deg ∈ [0°, 360°)`
  - `0°` = 阵列坐标系 +x 方向
  - `90°` = +y 方向，逆时针为正
  - `azimuth_offset_deg` 用于把阵列坐标系旋转到机器人坐标系
  - `microphones[i]` 必须对应输入流中的第 `i` 个通道；硬件通道顺序不同就改 `microphones` 或在采集侧重排，不要假设自动匹配
  - `microphones` 不能为空；3 麦等边阵列优先用 `CreateEquilateralTriangleConfig()` / `create_equilateral_triangle_config()` 生成配置
  - 当前版本验证重点是 N=3 等边阵列；N>3 代码路径保持通用，但应按具体硬件重新校准和验证

`MultiSoundLocator::CreateEquilateralTriangleConfig(side)` 默认生成 3 麦等边三角阵列：

```text
                 +x / 0 deg
                    mic0
                     *
                    / \
                   /   \
          mic1  *---+---*  mic2
               +y       -y
```

---

## C++ API

```cpp
#include "doa_service.h"

namespace SpacemitAudio {

// =============================================================================
// SoundLocatorConfig - 双声道配置
// =============================================================================

struct SoundLocatorConfig {
    int sample_rate = 16000;            // 采样率
    float mic_distance = 0.058f;        // 麦克风间距，单位米，必须匹配硬件
    float sound_speed = 343.0f;         // 声速，单位 m/s
    int fft_size = 0;                   // FFT 长度，0 = 自动
    int frame_size = 512;               // 每帧样本数
    int avg_frames = 4;                 // 输出一个结果前平均的帧数
    float confidence_threshold = 0.1f;  // 有效结果阈值
    int upsample_factor = 0;            // GCC 上采样因子，0 = 自动
    bool use_fftw_measure = false;      // 是否使用 FFTW_MEASURE
};

// =============================================================================
// SoundLocator - 双声道 GCC-PHAT
// =============================================================================

class SoundLocator {
public:
    explicit SoundLocator(const SoundLocatorConfig& config = {});

    bool Initialize();
    void Reset();

    bool Process(const float* interleaved, size_t num_frames);
    bool Process(const float* ch0, const float* ch1, size_t num_frames);
    bool Process(const int16_t* interleaved, size_t num_frames);

    float GetTDOA() const;              // 秒
    float GetDOA() const;               // 角度，[0, 180]
    float GetConfidence() const;        // [0, 1]
    bool IsValid() const;
    float GetAverageDOA() const;        // confidence 加权平均
    int GetResultCount() const;

    const SoundLocatorConfig& GetConfig() const;
    int GetMaxDelaySamples() const;
};

// =============================================================================
// MultiSoundLocatorConfig - 3+ 声道配置
// =============================================================================

struct MicrophonePosition {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;  // 当前版本只使用 x/y，z 预留
};

struct MultiSoundLocatorConfig {
    int sample_rate = 16000;
    int frame_size = 512;
    int avg_frames = 4;
    float max_avg_seconds = 0.0f;       // 0 = 自 Reset 起无界累积
    int fft_size = 0;
    int upsample_factor = 0;
    float confidence_threshold = 0.1f;
    float margin_threshold = 0.6f;
    float min_signal_rms = 0.0f;
    float quality_threshold = 0.0f;
    float closure_threshold_samples = 0.0f;
    float closure_threshold_fraction = 0.3f;
    float sound_speed = 343.0f;
    bool use_fftw_measure = false;
    std::vector<MicrophonePosition> microphones;
    float azimuth_offset_deg = 0.0f;
    float max_frequency_hz = 0.0f;      // 0 = 自动 alias-safe 频带上限
    float search_step_deg = 1.0f;       // 预留给 SRP-PHAT 变体
};

// =============================================================================
// MultiSoundLocatorResult - 3+ 声道结果
// =============================================================================

struct MultiSoundLocatorResult {
    float azimuth_deg = 0.0f;           // 机器人坐标系方位角，[0, 360)
    float confidence = 0.0f;            // 几何平均 pair peak * quality
    float peak_score = 0.0f;            // 旧语义：pair peak 算术平均
    float score_margin = 0.0f;          // 未归一化波前向量范数
    float quality = 0.0f;               // clamped consistency，[0, 1]
    float closure_residual_sec = 0.0f;  // N=3 时的 TDOA 闭环残差
    int valid_pairs = 0;
    bool valid = false;
};

// =============================================================================
// MultiSoundLocator - 3+ 声道平面 MPCC-LSQ
// =============================================================================

class MultiSoundLocator {
public:
    explicit MultiSoundLocator(const MultiSoundLocatorConfig& config);

    bool Initialize();
    void Reset();

    bool Process(const float* interleaved, size_t n_frames, int n_channels);
    bool Process(const float* const* channel_ptrs, size_t n_frames, int n_channels);
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

    float GetClosureResidual() const;        // 秒
    float GetClosureSamples() const;         // 样本数
    void GetPairConfidences(float* out, int n) const;
    int GetPairCount() const;
    float GetQuality() const;
    float GetAverageResultantLength() const; // [0, 1]，方向稳定性

    static MultiSoundLocatorConfig CreateEquilateralTriangleConfig(
        float side_length_m = 0.063f);
};

}  // namespace SpacemitAudio
```

### 生命周期

1. 构造 config。
2. 构造 locator。
3. 调用 `Initialize()`，失败时不要继续处理音频。
4. 循环调用 `Process()`，返回 `true` 表示已有一个 ready batch。
5. 读取 `GetDOA()` / `GetResult()` / 平均值。
6. 需要清空历史时调用 `Reset()`。

`Process()` 在未 `Initialize()` 时返回 `false`。`MultiSoundLocator::Process()` 的 `n_channels` 和 `config.microphones.size()` 不一致时会抛 `std::invalid_argument`。

---

## C++ 示例

### 双声道 PCM16

```cpp
#include <cstdint>
#include <vector>
#include "doa_service.h"

using namespace SpacemitAudio;

int main() {
    SoundLocatorConfig cfg;
    cfg.sample_rate = 16000;
    cfg.mic_distance = 0.058f;  // 必须按真实双麦间距设置
    cfg.frame_size = 512;
    cfg.avg_frames = 4;

    SoundLocator loc(cfg);
    if (!loc.Initialize()) {
        return 1;
    }

    std::vector<int16_t> pcm16_interleaved;  // [ch0, ch1, ch0, ch1, ...]
    size_t frames = pcm16_interleaved.size() / 2;

    if (loc.Process(pcm16_interleaved.data(), frames) && loc.IsValid()) {
        float doa = loc.GetDOA();
        float confidence = loc.GetConfidence();
        (void)doa;
        (void)confidence;
    }

    return 0;
}
```

### 三声道等边阵列

```cpp
#include <cstdint>
#include <vector>
#include "doa_service.h"

using namespace SpacemitAudio;

int main() {
    auto cfg = MultiSoundLocator::CreateEquilateralTriangleConfig(0.063f);
    cfg.sample_rate = 16000;
    cfg.frame_size = 512;
    cfg.avg_frames = 4;
    cfg.max_avg_seconds = 10.0f;  // live/长会话必须设置正值或定期 Reset()
    cfg.azimuth_offset_deg = 0.0f;

    MultiSoundLocator loc(cfg);
    if (!loc.Initialize()) {
        return 1;
    }

    std::vector<int16_t> pcm16_interleaved;  // [ch0, ch1, ch2, ch0, ...]
    size_t frames = pcm16_interleaved.size() / 3;

    if (loc.Process(pcm16_interleaved.data(), frames, 3)) {
        MultiSoundLocatorResult result = loc.GetResult();
        if (result.valid) {
            float azimuth = result.azimuth_deg;
            float quality = result.quality;
            (void)azimuth;
            (void)quality;
        }
    }

    return 0;
}
```

---

## Python API

Python 包名为 `spacemit_audio_process`。

```python
from spacemit_audio_process import (
    SoundLocator,
    SoundLocatorConfig,
    MicrophonePosition,
    MultiSoundLocatorConfig,
    MultiSoundLocatorResult,
    MultiSoundLocator,
)
```

### SoundLocator

```python
class SoundLocatorConfig:
    sample_rate: int
    mic_distance: float
    sound_speed: float
    fft_size: int
    frame_size: int
    avg_frames: int
    confidence_threshold: float
    upsample_factor: int
    use_fftw_measure: bool

class SoundLocator:
    def __init__(self, config: SoundLocatorConfig = ...): ...
    def initialize(self) -> bool: ...
    def reset(self) -> None: ...

    def process(self, interleaved: np.ndarray) -> bool: ...
    def process_int16(self, interleaved: np.ndarray) -> bool: ...
    def process_separate(self, ch0: np.ndarray, ch1: np.ndarray) -> bool: ...

    @property
    def tdoa(self) -> float: ...
    @property
    def doa(self) -> float: ...
    @property
    def confidence(self) -> float: ...
    @property
    def is_valid(self) -> bool: ...
    @property
    def average_doa(self) -> float: ...
    @property
    def result_count(self) -> int: ...
    @property
    def config(self) -> SoundLocatorConfig: ...
    @property
    def max_delay_samples(self) -> int: ...
```

`process()` 接收 1-D float32 交错数组 `[ch0, ch1, ch0, ch1, ...]`。`process_int16()` 接收 1-D int16 交错数组。`process_separate()` 接收两个长度相同的 1-D float32 数组。

### MultiSoundLocator

```python
class MicrophonePosition:
    def __init__(self, x: float, y: float, z: float = 0.0): ...

class MultiSoundLocatorConfig:
    sample_rate: int
    frame_size: int
    avg_frames: int
    max_avg_seconds: float
    fft_size: int
    upsample_factor: int
    confidence_threshold: float
    margin_threshold: float
    min_signal_rms: float
    quality_threshold: float
    closure_threshold_samples: float
    closure_threshold_fraction: float
    sound_speed: float
    use_fftw_measure: bool
    microphones: list[MicrophonePosition]
    azimuth_offset_deg: float
    max_frequency_hz: float
    search_step_deg: float

class MultiSoundLocatorResult:
    azimuth_deg: float
    confidence: float
    peak_score: float
    score_margin: float
    quality: float
    closure_residual_sec: float
    valid_pairs: int
    valid: bool

class MultiSoundLocator:
    @staticmethod
    def create_equilateral_triangle_config(
        side_length_m: float = 0.063,
    ) -> MultiSoundLocatorConfig: ...

    def __init__(self, config: MultiSoundLocatorConfig): ...
    def initialize(self) -> bool: ...
    def reset(self) -> None: ...

    def process(self, samples: np.ndarray, n_channels: int = 0) -> bool: ...
    def process_int16(self, samples: np.ndarray, n_channels: int = 0) -> bool: ...
    def process_channels(self, channels: Sequence[np.ndarray]) -> bool: ...

    @property
    def result(self) -> MultiSoundLocatorResult: ...
    @property
    def azimuth(self) -> float: ...
    @property
    def confidence(self) -> float: ...
    @property
    def is_valid(self) -> bool: ...
    @property
    def average_azimuth(self) -> float: ...
    @property
    def result_count(self) -> int: ...
    @property
    def closure_residual(self) -> float: ...
    @property
    def closure_samples(self) -> float: ...
    @property
    def pair_count(self) -> int: ...
    @property
    def pair_confidences(self) -> np.ndarray: ...
    @property
    def quality(self) -> float: ...
    @property
    def average_resultant_length(self) -> float: ...

    def get_tdoa(self, i: int, j: int) -> float: ...
    def get_max_delay_samples_pair(self, i: int, j: int) -> int: ...
```

实际使用 `MultiSoundLocator` 时必须传入包含 `microphones` 的配置。虽然 binding 层允许省略 `config`，默认配置没有麦克风几何，`initialize()` 会返回 `False`。

`process()` / `process_int16()` 支持两种输入：

- 2-D 数组：shape 为 `(frames, channels)`，`n_channels` 可省略。
- 1-D 交错数组：必须显式传 `n_channels`。

`process_channels()` 接收多个 1-D float32 通道数组，例如 `[ch0, ch1, ch2]`。

---

## Python 示例

### 双声道

```python
import numpy as np
from spacemit_audio_process import SoundLocator, SoundLocatorConfig

cfg = SoundLocatorConfig()
cfg.sample_rate = 16000
cfg.mic_distance = 0.058
cfg.frame_size = 512
cfg.avg_frames = 4

loc = SoundLocator(cfg)
assert loc.initialize()

pcm16 = np.zeros(512 * 2, dtype=np.int16)  # [ch0, ch1, ch0, ch1, ...]
if loc.process_int16(pcm16) and loc.is_valid:
    print(f"DOA: {loc.doa:.1f}, confidence={loc.confidence:.3f}")
```

### 三声道

```python
import numpy as np
from spacemit_audio_process import MultiSoundLocator

cfg = MultiSoundLocator.create_equilateral_triangle_config(0.063)
cfg.sample_rate = 16000
cfg.frame_size = 512
cfg.avg_frames = 4
cfg.max_avg_seconds = 10.0

loc = MultiSoundLocator(cfg)
assert loc.initialize()

pcm16 = np.zeros((512, 3), dtype=np.int16)  # shape: frames, channels
if loc.process_int16(pcm16):
    result = loc.result
    if result.valid:
        print(
            f"azimuth={result.azimuth_deg:.1f}, "
            f"confidence={result.confidence:.3f}, "
            f"quality={result.quality:.3f}"
        )
```

### 自定义麦克风几何

```python
from spacemit_audio_process import (
    MicrophonePosition,
    MultiSoundLocator,
    MultiSoundLocatorConfig,
)

cfg = MultiSoundLocatorConfig()
cfg.sample_rate = 16000
cfg.microphones = [
    MicrophonePosition(0.025, 0.025),
    MicrophonePosition(-0.025, 0.025),
    MicrophonePosition(-0.025, -0.025),
    MicrophonePosition(0.025, -0.025),
]
cfg.azimuth_offset_deg = 0.0
cfg.max_avg_seconds = 10.0

loc = MultiSoundLocator(cfg)
```

---

## 数据格式

- 默认采样率：`16000 Hz`
- 默认帧长：`512 samples`
- 默认平均：`4 frames` 输出一个 ready result
- float 输入：`float32`，建议范围 `[-1.0, 1.0]`
- PCM 输入：`int16` little-endian
- 交错格式：
  - 2ch: `[ch0, ch1, ch0, ch1, ...]`
  - 3ch: `[ch0, ch1, ch2, ch0, ch1, ch2, ...]`
- Python 2-D multi 输入：`(frames, channels)`

```python
# PCM16 bytes -> int16 interleaved
pcm16 = np.frombuffer(pcm_bytes, dtype=np.int16)

# int16 -> float32
audio_float = pcm16.astype(np.float32) / 32768.0
```

---

## 有效性与阈值

### SoundLocator

`IsValid()` 等价于 `GetConfidence() >= confidence_threshold`。`GetAverageDOA()` 只累计有效 batch。

### MultiSoundLocator

`result.valid` 至少受下面条件影响：

- `confidence >= confidence_threshold`
- `score_margin >= margin_threshold`
- `frame RMS >= min_signal_rms`
- `quality >= quality_threshold`
- N=3 时，closure samples 不超过有效 closure 阈值

N=3 的 closure 有效阈值为：

```text
max(
  closure_threshold_samples,
  closure_threshold_fraction * max_physical_TDOA_samples
)
```

将 `closure_threshold_samples <= 0` 且 `closure_threshold_fraction <= 0` 可关闭 closure gate。live 场景建议设置 `min_signal_rms`，否则静音/低能量噪声可能产生看似稳定的 GCC 峰。

`confidence` 是 pair GCC peak 的几何平均乘以 `quality`。如果旧代码曾按 v1.1 之前的算术平均调阈值，读取 `peak_score` 或重新校准 `confidence_threshold`。

---

## Demo 与验证

统一 demo 为 `ssl_demo`，`-c 2` 使用 `SoundLocator`，`-c 3+` 使用 `MultiSoundLocator`。

```bash
# SDK 内构建
source build/envsetup.sh
cd components/multimedia/audio_process
mm

# 2ch 合成信号
ssl_demo -c 2 -t -d 0.058

# 3ch 合成扫角
ssl_demo -c 3 -t -d 0.063 --sweep 0:330:30

# WAV 文件，-f 模式可按 WAV header 自动判断通道数
ssl_demo -f any.wav -d 0.063

# 3ch live，多麦设备存在且已确认通道映射时使用
ssl_demo -c 3 -l -d 0.063 -r 16000 -i 0 \
    --capture-channels 4 --pick 2,3,4 --avg-seconds 3 -v
```

PR scope 测试不依赖真实麦克风：

```bash
./scripts/test/robot-test run components/multimedia/audio_process/doa --scope pr
```

真实麦克风 smoke 需要人工确认设备、通道数、采样率和麦克风几何后再运行：

```bash
DOA_MANUAL_RUN=1 \
DOA_MANUAL_CHANNELS=3 \
DOA_MANUAL_CAPTURE_CHANNELS=4 \
DOA_MANUAL_PICK=2,3,4 \
./scripts/test/robot-test run components/multimedia/audio_process/doa --scope manual
```

---

## 集成与依赖

- 公开头文件：`include/doa_service.h`
- C++ 库：`sound_locator`
- Python 包：`spacemit_audio_process`
- 必需系统依赖：FFTW3f（`pkg-config --exists fftw3f`）
- C++ 标准：C++17

Standalone CMake 示例：

```bash
cd components/multimedia/audio_process/doa
cmake -S . -B build -DAUDIO_PROCESS_BUILD_EXAMPLES=ON -DAUDIO_PROCESS_BUILD_PYTHON=ON
cmake --build build -j"$(nproc)"
```

应用侧 CMake 示例：

```cmake
add_subdirectory(components/multimedia/audio_process/doa)
target_link_libraries(your_target PRIVATE sound_locator)
target_include_directories(your_target PRIVATE
    components/multimedia/audio_process/doa/include)
```

---

## 常见问题

**角度是期望值的补角**

2ch 通常是通道映射相反，可在 demo 用 `--flip` 验证，应用里 swap 通道或自行转换 `180 - doa`。3+ch 不使用 `--flip`，应通过 `azimuth_offset_deg` 或正确的 `microphones` 顺序修正阵列到机器人坐标系的映射。

**端火角 0° / 180° 附近误差大**

优先检查 `mic_distance`。端火角对麦克风间距误差很敏感，几毫米偏差就会让 0° / 180° 明显漂移。

**3ch 一直没有 valid result**

检查 `quality`、`closure_residual_sec`、`pair_confidences` 和 `min_signal_rms`。常见原因包括通道映射错误、麦克风坐标错误、通道间不同步、静音噪声、混响或多声源。

**live 长时间运行内存增长**

`MultiSoundLocatorConfig.max_avg_seconds = 0` 表示从上次 `Reset()` 起无界累计，只适合一次性文件或合成测试。实时麦克风或长会话必须设置正值，例如 `5.0` 到 `30.0`，或定期调用 `Reset()`。
