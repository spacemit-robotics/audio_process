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

**合成信号精度测试：**
```bash
ssl_demo -t -d 0.058
```

**WAV 文件测试：**
```bash
ssl_demo -f stereo.wav -d 0.058
```

如果硬件通道方向与期望相反，可加 `--flip` 输出补角：
```bash
ssl_demo -f stereo.wav -d 0.058 --flip
```

**Python 示例**（需已安装 Python 包或设置 PYTHONPATH 指向 SDK 构建产物）：
```bash
python -c "from spacemit_audio_process import SoundLocator; print('OK')"
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
./build/bin/ssl_demo -t -d 0.058
```

**WAV 文件测试：**
```bash
./build/bin/ssl_demo -f stereo.wav -d 0.058
```

如果需要让 0° 输出为 180°、50° 输出为 130°，可加 `--flip`：
```bash
./build/bin/ssl_demo -f stereo.wav -d 0.058 --flip
```

**Python 绑定安装与测试：**
```bash
make -C build audio_process-install-python
python -c "from spacemit_audio_process import SoundLocator; print('OK')"
```

**Python 示例（需先安装绑定）：**
```bash
# 合成信号精度测试
python python/examples/ssl_demo.py -t -d 0.058

# WAV 文件测试（逐帧输出）
python python/examples/ssl_demo.py -f stereo.wav -d 0.058 -v

# WAV 文件测试并翻转输出角度
python python/examples/ssl_demo.py -f stereo.wav -d 0.058 --flip

# 实时麦克风（需先安装 spacemit-audio）
# python -m pip install spacemit-audio \
#     --index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
python python/examples/ssl_demo.py -l -d 0.058
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


## 3.4 三声道 DOA (MultiSoundLocator)

`MultiSoundLocator` 是新增的多声道声源定位 API，和原有双声道 `SoundLocator` 并行存在。v1 固定采用 MPCC-LSQ：先对每个麦克风对执行 GCC-PHAT 得到 TDOA，再用麦克风几何矩阵做闭式最小二乘求解平面波方向，输出机器人坐标系下 `[0°, 360°)` 方位角。

**C++ API 概览**：

```cpp
#include "multi_sound_locator.h"

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

**合成信号验证命令**：

```bash
# C++：12 点扫角
./build/bin/multi_ssl_demo -t --geometry equilateral --mic-distance 0.063 --sweep 0:330:30

# C++：前后向区分
./build/bin/multi_ssl_demo -t --angle 45
./build/bin/multi_ssl_demo -t --angle 315

# Python：默认验收角度集合
python python/examples/multi_ssl_demo.py -t -d 0.063
```

**已知限制**：

- v1 只估计平面方位角，不估计 elevation；`MicrophonePosition.z` 暂不参与计算。
- v1 验证目标是 `N=3` 等边三角阵列；实现循环对 `N>3` 保持通用，但未做真实数据或系统验收。
- `multi_ssl_demo -f` 已支持 3 声道 PCM16 WAV 输入，但当前没有已标注的真实录音回归集。
- 如果 `score_margin` 明显偏低或 `valid_pairs` 长期不足，优先检查通道映射、麦克风坐标、平面波假设以及声源是否处于强混响环境。


## 4. 常见问题

**Q: 端火角（0°/180°附近）误差很大？**

`mic_distance` 必须精确匹配物理硬件。端火角对间距误差极其敏感——间距偏差几毫米时，135° 等中间角度可能看起来仍然正常，但 0°/180° 会明显偏离。如果录到的稳定 TDOA 约为 τ，那么实际间距约为 `c × |τ|`（例如 |TDOA| ≈ 168μs 对应间距 ≈ 0.0576m）。

**Q: DOA 结果是期望角度的补角（180° - 期望）？**

这是硬件通道映射问题。不同硬件的 ch0/ch1 物理位置不同。
C++/Python `ssl_demo` 可加 `--flip` 输出 `180 - doa`；
集成应用中也可以 swap 通道或自行使用 `180 - doa`。

**Q: 一直无有效结果？**

置信度低于阈值或输入不是双声道。检查 WAV 声道数，降低 `confidence_threshold` 做验证。

**Q: 结果抖动严重？**

单帧噪声或混响影响。增大 `avg_frames`，或对输出做业务侧平滑。

## 5. 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本   | 说明 |
| ------ | ---- |
| 1.0.0  | 提供 C++ / Python 接口，支持 GCC-PHAT 双声道声源定位。 |

## 6. 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：C++ 代码遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)。
- **提交前检查**：在仓库根目录运行 `bash scripts/lint/lint_cpp.sh components/multimedia/audio_process` 并确保通过。

## 7. License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。

## 8. 附录：算法流水线

```
双声道输入 → Hann 窗口 → 零填充至 2×frame_size → FFT
  → 互功率谱 → PHAT 加权 → 频域零填充（×16 上采样）
  → IFFT → 多帧平均 → ±max_delay 峰值搜索 → 抛物线插值
  → TDOA → DOA [0°, 180°]
```

关键常量：`kUpsampleFactor = 16`，在 16kHz 采样率下提供 ≈ 3.9μs 的时延分辨率。
