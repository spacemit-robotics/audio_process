# 声源定位（DOA）组件

## 1. 项目简介

本组件为双声道声源定位算法库，基于 GCC-PHAT（Generalized Cross-Correlation with Phase Transform）实现，提供统一的 C++ 接口与 Python 绑定，便于集成到机器人、智能音箱等需要声源方向感知的应用中。功能特性如下：

| 类别     | 支持                                                                 |
| -------- | -------------------------------------------------------------------- |
| 算法     | GCC-PHAT，零填充 FFT + PHAT 加权 + 频域上采样 + 抛物线插值 + 多帧平均 |
| 输入格式 | float32 交错、float32 分离双声道、int16 交错                         |
| 输出     | DOA 角度 [0°, 180°]（90°=正前方）、TDOA（秒）、置信度               |
| 精度     | 中间角度目标误差 < 3%，端火附近按绝对角度误差评估（宽带信号）        |
| 接口     | C++（`include/doa_service.h`）、Python（`spacemit_audio_process`）      |

## 2. 验证

按以下顺序完成依赖安装与示例运行。本组件为纯算法库，无需下载模型。

### 2.1. 安装依赖

- **编译环境**：CMake ≥ 3.16，C++17 编译器（GCC/Clang）。
- **必选**：FFTW3f（float 版本）。

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libfftw3-dev
```

**可选：**
- **Python 绑定**：`pip install pybind11` 或 `apt install python3-pybind11`

### 2.2. 测试

本节提供示例程序的编译与运行方式，便于开发者快速验证效果。使用前需先按下列两种方式之一完成编译，再运行对应示例。

- **在 SDK 中验证**（2.2.1）：在已拉取的 SpacemiT Robot SDK 工程内用 `mm` 编译，产物部署到 `output/staging`，适合整机集成或与其他模块联调。
- **独立构建下验证**（2.2.2）：在组件目录下用 CMake 本地编译，不依赖完整 SDK，适合快速体验。

#### 2.2.1. 在 SDK 中验证

**编译**：本组件已纳入 SpacemiT Robot SDK 时，在 SDK 根目录下执行。SDK 拉取与初始化见 [SpacemiT Robot SDK Manifest](https://github.com/spacemit-robotics/manifest)（使用 repo 时需先完成 `repo init`、`repo sync` 等）。

```bash
source build/envsetup.sh
cd components/multimedia/audio_process
mm
```

构建产物会安装到 `output/staging`。

**运行**：运行前在 SDK 根目录执行 `source build/envsetup.sh`，使 PATH 与库路径指向 `output/staging`，然后可执行：

> `ssl_demo` v1.1 起是单一 binary，按 `-c N` 选择算法路径（N=2 → `SoundLocator`；N≥3 → `MultiSoundLocator`）。`-f WAV` 模式下省略 `-c` 时会按 WAV 头自动判定声道数。

**合成信号精度测试：**
```bash
# 2-ch (SoundLocator, DOA ∈ [0°, 180°])
ssl_demo -c 2 -t -d 0.058

# 3-ch (MultiSoundLocator, azimuth ∈ [0°, 360°)，默认等边 0.063 m 阵列)
ssl_demo -c 3 -t -d 0.063 --sweep 0:330:30
```

**WAV 文件测试：**
```bash
ssl_demo -c 2 -f stereo.wav -d 0.058              # 2-ch
ssl_demo -c 3 -f 3ch.wav    -d 0.063              # 3-ch
ssl_demo       -f any.wav   -d 0.063              # auto from WAV header
```

如果硬件通道方向与期望相反：
```bash
ssl_demo -c 2 -f stereo.wav -d 0.058 --flip                          # 2-ch: 输出 180 − DOA
ssl_demo -c 3 -f 3ch.wav    -d 0.063 --azimuth-offset 180            # 3-ch: 旋转阵列坐标系
```

**Python 示例**（需已安装 Python 包或设置 PYTHONPATH 指向 SDK 构建产物）：
```bash
python -c "from spacemit_audio_process import SoundLocator, MultiSoundLocator; print('OK')"
```

#### 2.2.2. 独立构建下验证

在组件目录下完成编译后，运行下列示例。

**编译（含示例与 Python 绑定）：**
```bash
cd /path/to/audio_process/doa
cmake -S . -B build -DAUDIO_PROCESS_BUILD_EXAMPLES=ON -DAUDIO_PROCESS_BUILD_PYTHON=ON
cmake --build build -j$(nproc)
```

**合成信号精度测试：**
```bash
./build/bin/ssl_demo -c 2 -t -d 0.058                                 # 2-ch
./build/bin/ssl_demo -c 3 -t -d 0.063 --sweep 0:330:30                # 3-ch 12 角度
./build/bin/ssl_demo -c 3 -t -d 0.063 --angle 45                      # 3-ch 单角度
```

**WAV 文件测试：**
```bash
./build/bin/ssl_demo -c 2 -f stereo.wav -d 0.058
./build/bin/ssl_demo -c 3 -f 3ch.wav    -d 0.063
./build/bin/ssl_demo       -f any.wav   -d 0.063   # auto from header
```

如果需要让 0° 输出为 180°、50° 输出为 130°，2-ch 加 `--flip`；3-ch 用 `--azimuth-offset`：
```bash
./build/bin/ssl_demo -c 2 -f stereo.wav -d 0.058 --flip
./build/bin/ssl_demo -c 3 -f 3ch.wav    -d 0.063 --azimuth-offset 180
```

**实时麦克风（`-l`，需构建时找到 spacemit_audio）：**

`-r` 必须等于采集设备的实际采样率（用 `cat /proc/asound/card<N>/stream0`
或 `arecord -l` 查）。设备相关：板载 codec `snd-es8326` 只支持 48000；
USB 麦克风阵列常见 16000。`--avg-seconds` 在 live 下务必设正值（默认 0 是
无界累积）。
```bash
# 3 路麦克风直接采集（采样率按设备填，例：USB 16k）
./build/bin/ssl_demo -c 3 -l -d 0.063 -r 16000 --avg-seconds 3

# 4 路麦克风：ch1 = AEC 参考(丢弃)，ch2/3/4 = DOA 三路。
# 已在 K3 实测：USB SPV 4-mic，16000 Hz，device 0。
./build/bin/ssl_demo -c 3 -l -d 0.063 -r 16000 -i 0 \
    --capture-channels 4 --pick 2,3,4 --avg-seconds 3 -v
```

**Python 绑定安装与测试：**
```bash
make -C build audio_process-install-python
python -c "from spacemit_audio_process import SoundLocator, MultiSoundLocator; print('OK')"
```

**Python 示例（v1.1 起 Python demo 也合并为单一 `ssl_demo.py`，按 `-c N` 路由 —— 与 C++ 端一致）：**
```bash
# 2-ch
python python/examples/ssl_demo.py -c 2 -t -d 0.058
python python/examples/ssl_demo.py -c 2 -f stereo.wav -d 0.058 -v
python python/examples/ssl_demo.py -c 2 -f stereo.wav -d 0.058 --flip

# 3-ch
python python/examples/ssl_demo.py -c 3 -t -d 0.063 --sweep 0:330:30
python python/examples/ssl_demo.py -c 3 -f 3ch.wav -d 0.063 -v

# 实时麦克风（需先安装 spacemit-audio）
# python -m pip install spacemit-audio \
#     --index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
# 运行前先用 audio_demo list 或 spacemit_audio.AudioCapture.list_devices()
# 查询 SPV 的 PortAudio 输入设备索引，并把下面的 1 替换为实际查询结果。
# 变量赋值和 python 命令需在同一个 shell session 内执行。
SPV_INPUT_DEVICE=1
python python/examples/ssl_demo.py -c 2 -l -d 0.058
python python/examples/ssl_demo.py -c 3 -l -d 0.063
python python/examples/ssl_demo.py -c 3 -l -d 0.063 -r 16000 -i "$SPV_INPUT_DEVICE" \
    --capture-channels 4 --pick 2,3,4 --avg-seconds 3 -v
```

## 3. 应用开发

本章说明如何在自有工程中**集成声源定位并调用 API**。环境与依赖见 [2.1](#21-安装依赖)，编译与运行示例见 [2.2](#22-测试)。

### 3.1. 构建与集成产物

无论通过 [2.2.1](#221-在-sdk-中验证)（SDK）或 [2.2.2](#222-独立构建下验证)（独立构建）哪种方式编译，完成后**应用开发所需**的库与头文件如下，集成时只需**包含头文件并链接对应库**：

| 产物 | 说明 |
| ---- | ---- |
| `include/doa_service.h` | **C++ API 头文件**，应用侧只需包含此头文件并链接下方库即可调用 |
| `build/lib/libsound_locator.a` | C++ 核心库（静态），链接时使用 |
| `build/spacemit_audio_process/` | Python 包，`make audio_process-install-python` 安装后 `import spacemit_audio_process` |

示例可执行文件（非集成必需）：`build/bin/ssl_demo`。运行与验证步骤见 [2.2.1](#221-在-sdk-中验证) 或 [2.2.2](#222-独立构建下验证)。

### 3.2. API 使用

**C++**：头文件 `include/doa_service.h` 为唯一 API 入口，实现为 PIMPL。在业务代码中 `#include "doa_service.h"`，链接 `libsound_locator.a`（及 FFTW3f），即可使用。

```cpp
#include "doa_service.h"

SpacemitAudio::SoundLocatorConfig cfg;
cfg.mic_distance = 0.058f;  // 设置麦克风间距（米）
cfg.sample_rate = 16000;

SpacemitAudio::SoundLocator loc(cfg);
loc.Initialize();

// 喂入双声道 PCM16 数据
loc.Process(pcm16_stereo, num_frames);

if (loc.IsValid()) {
    printf("DOA: %.1f°\n", loc.GetDOA());
}

// 获取多批次加权平均 DOA
float avg = loc.GetAverageDOA();
```

**Python**：安装后 `from spacemit_audio_process import SoundLocator`。

```python
from spacemit_audio_process import SoundLocator, SoundLocatorConfig

cfg = SoundLocatorConfig()
cfg.mic_distance = 0.058

loc = SoundLocator(cfg)
loc.initialize()
loc.process(stereo_float32_array)

if loc.is_valid:
    print(f"DOA: {loc.doa:.1f}°")
```

**CMake 集成**：将本组件作为子目录引入，并链接 `sound_locator`、包含头文件路径即可。
```cmake
add_subdirectory(doa)
target_link_libraries(your_target PRIVATE sound_locator)
```

### 3.3. 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `sample_rate` | `int` | `16000` | 采样率（Hz） |
| `mic_distance` | `float` | `0.058` | 麦克风间距（米），**必须根据硬件精确设置** |
| `sound_speed` | `float` | `343.0` | 声速（m/s） |
| `frame_size` | `int` | `512` | 每帧采样点数（16kHz 下 ≈ 32ms） |
| `avg_frames` | `int` | `4` | 多帧平均数（约 128ms 窗口） |
| `fft_size` | `int` | `0` | FFT 长度；0 = 自动（2 × frame_size） |
| `confidence_threshold` | `float` | `0.1` | 置信度阈值，低于此值结果标记为无效 |
| `upsample_factor` | `int` | `0` | GCC 上采样因子；0 = 自动（目标 ≈ 4μs 时延分辨率） |

**`MultiSoundLocatorConfig`（3+ 声道）额外字段** —— 详细 API 见 §3.4。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `microphones` | `vector<MicrophonePosition{x,y,z}>` | `{}` | 麦克风坐标数组（米）；`N≥2`，v1 验证 `N=3` 等边三角；`z` 保留给未来 elevation |
| `azimuth_offset_deg` | `float` | `0.0` | 阵列坐标系→机器人坐标系的标量旋转偏移（度） |
| `max_avg_seconds` | `float` | `0.0` | `GetAverageAzimuth` 滑动窗长度（秒），`0` = 自 Reset 起 unbounded；`>0` 适合流式 / 运动声源 |
| `max_frequency_hz` | `float` | `0.0` | GCC-PHAT 频带上限；`0` = 自动取 `c / (2 × d_max)` alias-safe（等边 0.063 m 时约 2.72 kHz） |
| `margin_threshold` | `float` | `0.6` | `‖k̂_unnorm‖` 下限（raw norm 几何一致性）；静音/多源帧 `< 0.5` |
| `quality_threshold` | `float` | `0.0` | [A5] clamped quality `1 − \|1 − ‖k̂‖\|` 下限；`0` 关闭此 gate；推荐严格场景 `0.5` |
| `closure_threshold_samples` | `float` | `0.0` | [A2/P1.1] `\|τ_01+τ_12-τ_02\|`（样本数）显式上限；**仅 N=3 启用**；与 `closure_threshold_fraction` 取 max 后作为实际 gate；两者皆 `≤0` 关闭。**v1.1.1 BREAKING**：默认从 `2.0` 改为 `0.0`（fraction 接管）。|
| `closure_threshold_fraction` | `float` | `0.3` | [A2/P1.1] `\|closure\| / max_physical_TDOA` 上限分数；**仅 N=3 启用**；阵列尺度无关（0.063 m@16 kHz → effective ≈ 0.88 sample；0.21 m@16 kHz → effective ≈ 2.94 sample）。**v1.1.1 新字段**。|

**`MultiSoundLocatorResult` 新增字段**：

| 字段 | 含义 |
|------|------|
| `quality` | [A5] clamped 一致性 `[0, 1]`，1 = 完美平面波 |
| `closure_residual_sec` | [A1] TDOA 闭环残差（秒），仅 N=3，N≠3 时为 0 |
| `confidence` | [A4] **语义已变**：几何平均 × quality（不再是算术平均）。下游 Python 用旧阈值的话需要重新校准 |
| `peak_score` | [A4] 旧的算术平均，保留作 backward-compat —— 已用 `peak_score` 调过阈值的代码可保持不变 |

**新增 accessor / 属性（C++ + Python）**：

- `GetClosureResidual()` / `closure_residual` 属性（秒）
- `GetClosureSamples()` / `closure_samples` 属性（样本）
- `GetPairConfidences(out, n)` / `pair_confidences` 属性（numpy length-N_pairs）
- `GetPairCount()` / `pair_count`
- `GetQuality()` / `quality` 属性
- `GetAverageResultantLength()` / `average_resultant_length` 属性（Mardia mean resultant length, `[0, 1]`）


## 3.4 三声道 DOA (MultiSoundLocator)

`MultiSoundLocator` 是新增的多声道声源定位 API，和原有双声道 `SoundLocator` 并行存在。v1 固定采用 MPCC-LSQ：先对每个麦克风对执行 GCC-PHAT 得到 TDOA，再用麦克风几何矩阵做闭式最小二乘求解平面波方向，输出机器人坐标系下 `[0°, 360°)` 方位角。

**C++ API 概览**：

```cpp
#include "doa_service.h"

auto cfg = SpacemitAudio::MultiSoundLocator::CreateEquilateralTriangleConfig(0.063f);
cfg.sample_rate = 16000;
cfg.azimuth_offset_deg = 0.0f;

SpacemitAudio::MultiSoundLocator loc(cfg);
loc.Initialize();
loc.Process(pcm_or_float_interleaved, num_frames, 3);

SpacemitAudio::MultiSoundLocatorResult result = loc.GetResult();
if (result.valid) {
    printf("azimuth: %.1f\n", result.azimuth_deg);
}
```

**Python API 概览**：

```python
from spacemit_audio_process import MultiSoundLocator

cfg = MultiSoundLocator.create_equilateral_triangle_config(0.063)
loc = MultiSoundLocator(cfg)
loc.initialize()
loc.process(samples_float32_n_frames_by_3)
print(loc.result.azimuth_deg, loc.result.confidence)
```

**默认等边三角形几何**：质心在原点，`mic0` 位于机器人前方 `+x`，`mic1/mic2` 位于后方左右两侧，边长默认 `0.063 m`。

```text
                 +x forward / 0 deg
                       mic0
                        *
                       / \
                      /   \
             mic1 *---+---* mic2
                  +y       -y
```

| 约定/参数 | 含义 |
|-----------|------|
| `MicrophonePosition{x, y, z}` | 单位米；v1 只使用 `x/y`，`z` 保留给未来俯仰角 |
| `microphones` | 麦克风坐标数组；v1 支持并验证 `N=3` 等边三角，`N>3` 仅为实验性 |
| `azimuth_deg` | 输出范围 `[0°, 360°)` |
| `0°` | 机器人前方 `+x` |
| 正方向 | 逆时针 CCW |
| `azimuth_offset_deg` | 将阵列坐标系旋转到机器人坐标系的运行时偏移 |
| `max_frequency_hz` | GCC-PHAT 频带上限；`0` 自动取 `c / (2 * d_max)`，等边 `0.063 m` 时约 `2.72 kHz` |
| `score_margin` | MPCC-LSQ 中为未归一化波前向量范数，接近 `1` 表示 TDOA 互相一致 |
| `valid_pairs` | GCC 峰值超过置信度阈值的麦克风对数量 |

**合成信号验证命令**（v1.1 起 `ssl_demo` 是单一 binary，按 `-c N` 路由）：

```bash
# C++：12 点扫角
./build/bin/ssl_demo -c 3 -t --mic-distance 0.063 --sweep 0:330:30

# C++：前后向区分
./build/bin/ssl_demo -c 3 -t --angle 45  --mic-distance 0.063
./build/bin/ssl_demo -c 3 -t --angle 315 --mic-distance 0.063

# C++：自定义阵列几何（4 mic 示例，正方形 0.05 m）
./build/bin/ssl_demo -c 4 -t --positions '0.025,0.025;-0.025,0.025;-0.025,-0.025;0.025,-0.025' --angle 30

# C++：调节新加的可观测性阈值（A1/A2/A5 暴露的字段）
./build/bin/ssl_demo -c 3 -t -d 0.063 \
    --quality-threshold 0.5 \
    --closure-threshold-fraction 0.3 \
    --margin-threshold 0.6

# Python（v1.1 起 Python demo 也合并；按 -c N 路由）
python python/examples/ssl_demo.py -c 3 -t -d 0.063 --sweep 0:330:30
```

### 3.5 v1.1 升级注意（A4 BREAKING）

v1.1 改变了 `MultiSoundLocatorResult.confidence` 的语义（从算术平均改为几何平均 × quality）。如果你已经按旧值调阈值，做下面任意一种迁移：

```python
# 方案 A：读取保留的 `peak_score`（旧的算术平均），原阈值继续可用
result = locator.result
if result.peak_score >= 0.20:   # ← 用 peak_score 替代旧 confidence
    use(result.azimuth_deg)

# 方案 B：重校 confidence 阈值（新值通常比旧的低 5–15%）
if result.confidence >= 0.17:   # ← 之前是 0.20 的话，先按 ~0.85× 试
    use(result.azimuth_deg)
```

### 3.5.1 v1.1.1 升级注意（P1.1 BREAKING，multi 路径，2-ch 不变）

v1.1.1 修复两个 scale-drift 问题。**只影响 `MultiSoundLocator` (multi 路径)，2-ch `SoundLocator` 保持 bit-stable，无任何 behavior 变化。**

**BREAKING #1 — multi confidence 量纲改变 (~3× 提升)**：之前 multi 路径的 pair GCC 峰值是 raw IFFT 峰 × `1/padded_size`，量纲依赖 active bin 数。0.063 m 阵列 @16 kHz 带限 2.72 kHz → 175 active bins → 理想 peak ≈ 0.34（不是 header 文档说的 ≈1.0）。v1.1.1 起每对峰按 `(2·active_bins − 1) / padded_size` 归一化，clean 源 ≈ 1.0，跨 fft_size / band_limit / mic 几何 invariant。

由于 multi `confidence = geometric_mean(pair_peaks) × quality`，pair 峰提升 ~3× 后 `confidence` 整体也 ~2.5–3× 增加。**§3.5 给的 0.17 / 0.20 阈值建议作废**；如果你之前用 v1.1 的 multi 路径并调过阈值：

```python
# 方案 A（推荐）：阈值乘 ~3 (实际乘 1/old_ideal_peak = padded_size / (2*active_bins - 1))
# 0.063 m@16 kHz padded=1024 → 乘 1024/(2*175-1) ≈ 2.93
if result.confidence >= 0.50:   # 之前 v1.1 multi 是 0.17 的话
    use(result.azimuth_deg)

# 方案 B：直接用默认 0.1 (多数场景已经够严格)
config.confidence_threshold = 0.1   # multi 路径默认
```

**BREAKING #2 — closure_threshold 默认从 samples 转 fraction**：之前 `closure_threshold_samples = 2.0`（固定 samples）在 0.063 m@16 kHz 是 max physical TDOA 的 ~68%，几乎无 prune 作用。v1.1.1 默认 `samples = 0.0`、`fraction = 0.3`，effective = `max(samples, fraction * max_physical_TDOA_samples)`，阵列尺度 invariant。

迁移策略：
```python
# 方案 A（推荐）：用新默认 fraction-based gate
# 0.063 m@16 kHz → effective ≈ 0.88 sample
# 0.21 m@16 kHz → effective ≈ 2.94 sample
# 想 disable: 显式 samples=0, fraction=0

# 方案 B：保留原 v1.1 行为（gate 几乎不生效）
config.closure_threshold_samples = 2.0
config.closure_threshold_fraction = 0.0   # 显式关 fraction，回到固定 samples
```

新增可观测性字段（pybind 全部暴露）：

| C++ | Python | 含义 |
|---|---|---|
| `result.quality` | `loc.quality` / `loc.result.quality` | clamped `1 − \|1 − ‖k̂‖\|`，[0,1] |
| `result.closure_residual_sec` | `loc.closure_residual` | `\|τ_01+τ_12−τ_02\|` 秒（仅 N=3） |
| `GetClosureSamples()` | `loc.closure_samples` | 同上但单位为样本数 |
| `GetPairConfidences(out, n)` | `loc.pair_confidences` | length-N_pairs numpy array |
| `GetPairCount()` | `loc.pair_count` | N×(N−1)/2 |
| `GetAverageResultantLength()` | `loc.average_resultant_length` | Mardia 平均合成长度，[0,1]，方向稳定性 |

`MultiSoundLocatorConfig` 新增可调阈值：

| 字段 | 默认 | 说明 |
|---|---|---|
| `closure_threshold_samples` | `0.0`（v1.1.1 起；原 v1.1 `2.0`） | N=3 时 TDOA 闭环残差显式上限（样本数）；与 `fraction` 取 max |
| `closure_threshold_fraction` | `0.3`（v1.1.1 新增） | N=3 时 `\|closure\|/max_physical_TDOA` 上限；阵列尺度 invariant |
| `quality_threshold` | `0.0` | clamped quality 下限；`0` 关闭此 gate |

**已知限制**：

- v1 只估计平面方位角，不估计 elevation；`MicrophonePosition.z` 暂不参与计算。
- v1 验证目标是 `N=3` 等边三角阵列；实现循环对 `N>3` 保持通用，但未做真实数据或系统验收。
- `ssl_demo -c 3 -f` 已支持 3 声道 PCM16 WAV 输入，但当前没有已标注的真实录音回归集。
- 如果 `score_margin` 明显偏低或 `valid_pairs` 长期不足，优先检查通道映射、麦克风坐标、平面波假设以及声源是否处于强混响环境。
- `max_avg_seconds = 0`（默认）是 unbounded 累积，仅适合一次性扫角；live 麦克风 / 长会话务必设正值（如 `--avg-seconds 10`）防止内存增长。
- CLI 调 closure gate 时，`--closure-threshold-samples` 与 `--closure-threshold-fraction` 会取 max 后生效；如果要完全关闭 closure gate，两个参数都要设为 `0`。


## 4. 常见问题

**Q: 端火角（0°/180°附近）误差很大？**

`mic_distance` 必须精确匹配物理硬件。端火角对间距误差极其敏感——间距偏差几毫米时，135° 等中间角度可能看起来仍然正常，但 0°/180° 会明显偏离。如果录到的稳定 TDOA 约为 τ，那么实际间距约为 `c × |τ|`（例如 |TDOA| ≈ 168μs 对应间距 ≈ 0.0576m）。

**Q: DOA 结果是期望角度的补角（180° - 期望）？**

这是硬件通道映射问题。不同硬件的 ch0/ch1 物理位置不同。
- 2-ch (`-c 2`)：C++ / Python `ssl_demo` 加 `--flip` 输出 `180 − doa`；集成应用中可 swap 通道或自行 `180 − doa`。
- 3-ch+ (`-c 3+`)：用 `--azimuth-offset DEG` 旋转阵列坐标系到机器人正前 = 0°；不要在 3-ch 模式用 `--flip`（会报错）。集成应用中设 `MultiSoundLocatorConfig.azimuth_offset_deg`。

**Q: 一直无有效结果？**

- 2-ch：置信度低于阈值或输入不是双声道。检查 WAV 声道数，降低 `confidence_threshold` 做验证。
- 3-ch+：检查 `result.quality`（< 0.5 说明平面波假设破坏，常见于强混响 / 多源）、`result.closure_residual_sec`（应低于 effective closure threshold，对应 `max(samples, fraction × max_physical_TDOA_samples) / sample_rate`；否则可能硬件通道间 desync）、`result.pair_confidences`（某一对很低 = 该麦克风通道差）；v1.1 起 `confidence` 是几何平均 × quality，比旧的算术平均敏感。

**Q: 升级到 v1.1 后 `confidence` 数值整体变小？**

是预期行为（A4）。v1.1 起 `confidence = geometric_mean(pair_peaks) × quality`，比旧的算术平均更严格地惩罚单个 dead pair。详见 §3.5。如果暂时不想调阈值，读 `result.peak_score`（保留的旧算术平均）即可。

**Q: 结果抖动严重？**

单帧噪声或混响影响。增大 `avg_frames`，或对输出做业务侧平滑。

## 5. 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本   | 说明 |
| ------ | ---- |
| 1.0.0  | 提供 C++ / Python 接口，支持 GCC-PHAT 双声道声源定位。 |
| 1.1.0  | 新增 `MultiSoundLocator`（3+ 声道平面 MPCC-LSQ，azimuth ∈ [0°, 360°)）；C++ + Python 两端的 `ssl_demo` 合并为单一 binary / 脚本（按 `-c N` 路由）；暴露 closure / quality / pair_confidences / average_resultant_length 等新可观测性字段；公开头文件合并为 `doa_service.h` 一个；demo CLI 暴露 `--quality-threshold` / `--margin-threshold` / `--closure-threshold-samples` / `--max-frequency-hz`；缺 `-d` 时打 stderr 警告。**BREAKING**: `MultiSoundLocatorResult.confidence` 改为几何平均 × quality（旧的算术平均保留为 `peak_score`）。详细迁移见 §3.5。 |
| 1.1.1  | Runtime-safety 修复（multi 路径独立 BREAKING；2-ch `SoundLocator` 保持 bit-stable 不变）。**BREAKING #1**：multi 每对 GCC 峰按 `(2·active_bins − 1) / padded_size` 归一化（之前 raw 值随 fft_size / band_limit / mic 几何漂移），clean 源 ≈ 1.0 跨配置 invariant；multi `confidence` 整体 ~2.5–3× 提升，§3.5 的 0.17/0.20 阈值建议作废，迁移见 §3.5.1。**BREAKING #2**：`closure_threshold_samples` 默认从 `2.0` 改为 `0.0`，新增 `closure_threshold_fraction = 0.3`（effective = `max(samples, fraction × max_physical_TDOA_samples)`），阵列尺度 invariant；显式保留旧 behavior 见 §3.5.1。**修复**：Python 2-ch wrappers (`process_float` / `process_int16` / `process_separate`) 加 `c_style \| forcecast` flag 防 strided / fortran array silent-read 错位（与 multi wrappers 一致）；multi `Process` 改用 buffer reuse（`resize` 替代每次 `assign`），稳态零 realloc，匹配 2-ch `buf_a/buf_b` 模式，机器人 16 kHz 连续回调下音频线程零抖动。**新字段**：`MultiSoundLocatorConfig::closure_threshold_fraction`（pybind 暴露 `cfg.closure_threshold_fraction`）。 |

## 6. 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：C++ 代码遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)。
- **提交前检查**：在仓库根目录运行 `bash scripts/lint/lint_cpp.sh components/multimedia/audio_process` 并确保通过。

## 7. License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。

## 8. 附录：算法流水线

### 8.1 双声道（`SoundLocator`）

```
双声道输入 → Hann 窗口 → 零填充至 2×frame_size → FFT
  → 互功率谱 → PHAT 加权 → 频域零填充（×16 上采样）
  → IFFT → 多帧平均 → ±max_delay 峰值搜索 → 抛物线插值
  → TDOA → DOA [0°, 180°]
```

关键常量：`kUpsampleFactor = 16`，在 16kHz 采样率下提供 ≈ 3.9μs 的时延分辨率。

### 8.2 多声道（`MultiSoundLocator`，N≥2，平面 MPCC-LSQ）

```
N 声道输入 → 每 mic Hann + 零填充 + FFT  (一次 FFT/mic，跨 pair 复用)
  ┃
  ┣→ pair (i,j) PHAT 加权 + 频带限制 + 频域上采样 IFFT
  ┃       → 多帧平均 → 峰值 → TDOA τ_ij + GCC peak（pair_confidences[]）
  ┃   （C(N,2) 对各自一次；N=3 时 3 对）
  ┃
  ┣→ [A1/A2] N=3 时算 closure τ_01 + τ_12 − τ_02；
  ┃       超 max(closure_threshold_samples,
  ┃              closure_threshold_fraction × max_physical_TDOA_samples) 标 invalid
  ┃
  ┣→ MPCC-LSQ：A=[Δp_ij]，b=c·τ_ij；Cholesky 解 k̂ = (AᵀA)⁻¹Aᵀb
  ┃   （AᵀA 在 Initialize 一次性分解，每帧只做 forward/back substitution）
  ┃
  ┣→ azimuth = atan2(k̂_y, k̂_x) + azimuth_offset_deg, NormalizeAngle360 → [0°, 360°)
  ┣→ score_margin = ‖k̂‖ (raw)
  ┣→ [A5] quality = clamp(1 − |1 − ‖k̂‖|, 0, 1)
  ┣→ [A4] confidence = (Π pair_peak)^(1/N_pairs) × quality   # 几何平均 × quality
  ┣→ peak_score = mean(pair_peak)  # 旧的算术平均（backward-compat）
  ┃
  ┗→ valid = confidence ≥ confidence_threshold
          ∧ score_margin ≥ margin_threshold
          ∧ quality ≥ quality_threshold
          ∧ (N≠3 ∨ closure_samples ≤ effective_closure_threshold)
       ┗→ 若 valid：BatchEntry(weighted_x, weighted_y, weight=confidence) push 进 deque
              ├ average_x/y += weighted_x/y, sum_weights += confidence
              └ 若 max_avg_seconds > 0：evict 老 batch 时同步 -= 三个量
                   ↑ 保持 [A6] GetAverageResultantLength = ‖(avg_x, avg_y)‖ / sum_weights ∈ [0, 1] 的 O(1)
```

**FFT 算力优化**：HEAD 采用"FFT-per-mic + spectrum 入参"设计 —— N=3 时每帧只做 3 次 forward FFT（不是 6 次 per-pair-per-channel），跨 pair 复用频谱。N=4 收益更大（4 vs 12）。`gcc_phat_pair` 内部只拥有 IFFT plan 与 PHAT 累加器。

**几何 + 别名安全**：`max_frequency_hz` 默认自动取 `c / (2 × d_max)`，避免空间频率别名（等边 0.063 m → 约 2.72 kHz）。`microphones` 可变长 `N≥2`，几何在 `BuildGeometry()` 阶段 once-only 验证并预算 Cholesky 因子。
