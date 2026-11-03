# 1080p 实时畸变与旋转校正系统——R18 源码与烧录交付

## 工程入口
Pango Design Suite 2025.2，器件 PG2L200H-6FBB676。
解压到英文短路径并保留全部目录，打开：
`wanchengceshi/algorithm_v04_1080p_upgrade/algorithm1080_nn_r1/build_revision18_timing/algorithm1080_nn.pds`

## 内容
本包收录当前 R18 PDS 实际引用的 RTL、IP、引脚/时序约束、初始化表、CNN 权重与模型配置，以及配套 V5 控制台、画质检测、标定和 SFR 源码。
部分模块沿用早期文件名，属于当前工程或当前软件的实际依赖。源码中的功能检查、错误处理和会话校验保持原样。

## PC 软件
Windows、Python 3.12，安装 requirements.txt，FFmpeg/ffprobe 加入 PATH。
双击 01_Open_Control.cmd 或 02_Open_Inspector.cmd；可设置 FPGA_PYTHON 指定解释器。
LATENCY_GUIDE.md 提供检测软件的接口使用说明。
CNN 权重在 FPGA 工程 data 目录，控制台依据实际板端状态加载到 DDR；镜头/焦距变化时重新标定。

## 文件完整性
运行 `python check_source_package.py` 检查包内文件哈希和 108 项 PDS 输入路径。
SOURCE_MANIFEST.json 记录本包源文件清单；SHA256SUMS.txt 记录交付摘要。
源码保留现有相对路径适配，工具安装与 PDS 内置器件库由开发环境提供。

## 烧录文件与实现报告
programming 目录提供最终 .sbit、配套 .smsk 和位流生成报告；implementation_reports 目录提供原始时序、约束例外与资源报告。操作与摘要见《烧录与实现报告说明.md》，文件来源和摘要见 DELIVERY_FILES.json。
