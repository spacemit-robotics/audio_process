# Audio Process Components

## 1. 项目简介

`audio_process` 是 SpacemiT Robot SDK 的音频处理算法集合，面向机器人、语音交互和智能终端场景，承载与音频采集/播放不同层级的信号处理能力。

当前已包含 DOA（Direction of Arrival）声源定位组件。后续可在本仓库继续扩展 3A 音频处理算法及其他音频前处理能力，例如 AEC（回声消除）、ANS/NS（噪声抑制）、AGC（自动增益控制）等。

## 2. 组件目录

| 路径 | 状态 | 说明 |
| --- | --- | --- |
| `doa/` | 已支持 | 双声道声源定位，基于 GCC-PHAT 输出 DOA、TDOA 和置信度。 |

新增算法建议按独立子目录组织，并保持和 `doa/` 一致的结构：

```text
<algorithm>/
├── include/        # C/C++ public API
├── src/            # implementation
├── examples/       # demo tools
├── python/         # optional Python binding and examples
├── CMakeLists.txt
├── README.md
└── package.xml
```

## 3. 当前能力

### 3.1 DOA 声源定位

DOA 组件位于 `doa/`，提供 C++ API、Python 绑定和 `ssl_demo` 示例程序。

| 项目 | 说明 |
| --- | --- |
| C++ API | `doa/include/doa_service.h` |
| Python 包 | `spacemit_audio_process` |
| 示例程序 | `ssl_demo` |
| 算法 | GCC-PHAT，支持双声道 PCM16 / float32 输入 |

详细编译、运行和 API 说明见 [doa/README.md](doa/README.md)。

## 4. 构建方式

在 SpacemiT Robot SDK 根目录下加载环境后，进入对应算法目录构建：

```bash
source build/envsetup.sh
lunch kx-generic-omni_agent
cd components/multimedia/audio_process/doa
mm
```

需要 Python wheel 时：

```bash
cd components/multimedia/audio_process/doa
mm -py
```

`m -py` / `mm -py` 负责构建 wheel；运行 Python 示例前，请在虚拟环境中安装生成的 wheel 或发布包。

## 5. 开发约定

- 每个算法组件应提供清晰的 C/C++ public API，头文件放在组件自己的 `include/` 目录。
- 示例工具放在 `examples/`，用于快速验证算法输入、输出和性能。
- Python 绑定放在 `python/`，包名应保持稳定，便于上层应用调用。
- 依赖、构建命令、测试命令和硬件假设应写在各组件自己的 `README.md` 中。

## 6. License

本仓库遵循 Apache-2.0，详见 [LICENSE](LICENSE)。第三方依赖声明见 [NOTICE](NOTICE)。
