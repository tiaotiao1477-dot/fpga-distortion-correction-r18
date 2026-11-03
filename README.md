# 1080p 实时镜头畸变与旋转校正系统

本项目在紫光同创 **PG2L200H-6FBB676** FPGA 上处理 OV5640 摄像头的视频。R18 工程将摄像头采集、DDR3 帧缓存、镜头畸变与旋转校正接入 HDMI 输出。Windows 上位机通过串口设置旋转角度、加载 CNN 权重，并可配合采集设备比较校正前后的画质。

| 想做什么 | 从哪里开始 |
| --- | --- |
| 用现成位流上板 | [R18 烧录文件](programming/video1080_stream_top.sbit) · [烧录与实现报告说明](烧录与实现报告说明.md) |
| 查看或重新构建 FPGA 工程 | [R18 PDS 工程](wanchengceshi/algorithm_v04_1080p_upgrade/algorithm1080_nn_r1/build_revision18_timing/algorithm1080_nn.pds) |
| 调节角度、使用自动摆正 | [V5 控制台启动脚本](01_Open_Control.cmd) |
| 测量斜边清晰度、帧率或延迟 | [画质检测启动脚本](02_Open_Inspector.cmd) · [硬件延迟测量指南](sw/quality_inspector_v1/LATENCY_GUIDE.md) |

## 获取并校验交付文件

在 Windows PowerShell 中运行以下命令。PDS 工程通过相对路径引用其他目录，请保留仓库结构，优先使用不含空格或中文的短路径。

```powershell
git clone https://github.com/tiaotiao1477-dot/fpga-distortion-correction-r18.git C:\fpga\r18
cd C:\fpga\r18
python -B check_source_package.py
```

校验通过时会显示 `OK: source hashes and 108 current PDS input references verified`。脚本核对交付文件的 SHA-256，并逐项检查 PDS 输入路径。

## 上板或重新构建

**使用现成位流。** 给开发板上电，接好 JTAG 下载器，在 PDS 2025.2 配套的 Fabric Configuration 中扫描 PG2L200H。选择 `programming/video1080_stream_top.sbit` 执行 SRAM 下载，并让同目录的 `.smsk` 配套文件保留在原位。下载后检查工具结果和 DONE 状态。SRAM 配置在断电后需要重新下载。

**从源码构建。** 使用 Pango Design Suite 2025.2 打开上表中的 PDS 工程。确认目标器件为 `PG2L200H-6FBB676`，顶层为 `video1080_stream_top`，再依次运行综合、映射、布局布线、时序分析和位流生成。仓库保留了 R18 使用的供应商 IP 文件，器件库由 PDS 安装环境提供。

## 启动 Windows 上位机

安装 Python 3.12 和 FFmpeg，确保在 PowerShell 中能运行 `ffmpeg` 和 `ffprobe`。在仓库根目录安装依赖，以下设置会让两个启动脚本使用同一个 Python 环境。

```powershell
py -3.12 -m venv .venv
$env:FPGA_PYTHON = (Resolve-Path .\.venv\Scripts\python.exe).Path
& $env:FPGA_PYTHON -m pip install -r requirements.txt
```

在**同一个 PowerShell 窗口**中运行 `.\01_Open_Control.cmd` 打开控制台，选择实际串口和采集设备。使用 CNN 功能时，控制台会根据板端状态将随包权重加载到 DDR。更换镜头或焦距后，应重新取得标定参数。[相机标定工具](sw/calibration/calibrate_camera.py)

需要测量画质时，运行 `.\02_Open_Inspector.cmd`。进行硬件延迟测量前请先关闭旋转控制台，以免两个程序同时操作串口。[测量流程与结果字段](sw/quality_inspector_v1/LATENCY_GUIDE.md)

## 实现报告与复现范围

随包 R18 原始时序报告的结论为 **All Constraints Met**。报告列出的核心时钟为 125 MHz，余量为 +0.248 ns。重新构建后，可用本次生成的时序和资源报告与随包结果对照。[原始时序报告](implementation_reports/video1080_stream_top.rtr) · [资源占用报告](implementation_reports/video1080_stream_top.rur) · [位流与报告摘要](DELIVERY_FILES.json)

本仓库提供从现成 R18 工程开始构建所需的工程文件，以及可直接下载的位流。若要重跑 R16/R17 与 R18 的历史对照测试，还需取得旧版基线和对应测试脚本。[交付源码清单](SOURCE_MANIFEST.json) · [原交付说明](README_zh.md)
