# 犀牛派 A1 视觉跟随小车

> 基于 **犀牛派 A1**（AidLux）+ **STM32G071RB** 的两轮差速/履带视觉跟随机器人。上层在犀牛派 A1 上跑 SSDLite-MobileNetV3 人体检测 + 简易多目标跟踪，下层 STM32 通过 TB6612FNG 驱动 12 V 履带电机；两者之间用 USB-UART 走一个自定义 7 字节定长串口协议。

- 仓库：<https://gitee.com/h2705/rhino-pi-a1-robot-car>
- 开发板：犀牛派 A1（阿加犀生态）
- 案例类型：机器人 + 教育 / 智能跟随
- 开源协议：[MIT](#-开源协议)

---

## 目录

- [1. 设计理念与应用场景](#1-设计理念与应用场景)
- [2. 功能演示](#2-功能演示)
- [3. 硬件清单（BOM）](#3-硬件清单bom)
- [4. 系统总体架构](#4-系统总体架构)
- [5. 机械结构](#5-机械结构)
- [6. 硬件接线](#6-硬件接线)
- [7. 电机驱动板](#7-电机驱动板)
- [8. 软件架构](#8-软件架构)
  - [8.1 下位机：STM32G071 电机驱动](#81-下位机stm32g071-电机驱动)
  - [8.2 上位机：犀牛派 A1 视觉跟随](#82-上位机犀牛派-a1-视觉跟随)
  - [8.3 串口通信协议](#83-串口通信协议)
- [9. 复现步骤](#9-复现步骤)
- [10. 控制算法说明](#10-控制算法说明)
- [11. 常见问题（FAQ）](#11-常见问题faq)
- [12. 仓库目录结构](#12-仓库目录结构)
- [13. 开源协议](#13-开源协议)
- [14. 致谢与生态标识](#14-致谢与生态标识)

---

## 1. 设计理念与应用场景

这是一台**以犀牛派 A1 为视觉核心、STM32 为实时执行核心**的差速履带小车。设计目标：

- **让小车"跟人走"**：USB 摄像头实时采集画面，检测 COCO 数据集中的 `person` 类，用跟踪器锁定最近目标，再通过横向 PD + 纵向 PID 解算出左右轮 PWM，使小车始终把目标保持在画面中央、并维持一个安全的拍摄距离。
- **软硬解耦**：犀牛派 A1 只负责"看"和"决策"，STM32 只负责"按指令驱动电机"。两者通过 UART 解耦，替换视觉算法或更换底盘时都不需要重写驱动层。
- **教学友好**：BOM 全部为通用件（Nucleo 板 + 自制 TB6612 驱动板 + 标准履带电机），机械件是两块环氧树脂/亚克力板 + 3D 打印摄像头支架，可直接用仓库里的 STEP / 3mf 文件复刻。

典型应用场景：

- 教育机器人实验平台（视觉检测 / 跟踪 / PID 控制 / 嵌入式串口通信一条龙）
- 迎宾 / 导览小车的底盘原型
- "机器人 + 养老服务"中的陪行/跟随载体原型

---

## 2. 功能演示

> 按《开发者案例征集规范》要求，需补充 5–10 min 演示视频并发布到 B 站 / 抖音 / 小红书 / 视频号（至少一个）。以下为占位，发布后替换为真实链接：

- 效果演示视频（待补充）：<TODO: B 站 / 视频号链接>
- 关键步骤讲解（待补充）：<TODO: 链接>
- 带话题：`#犀牛派开发者案例` `#阿加犀生态`

当前仓库内已包含：

- `接线图/系统接线图.png` —— 整机接线总览
- `电机驱动板原理图/电机驱动板V1.0.pdf` —— 自制驱动板原理图
- `小车框架基板/上板.STEP`、`下板.STEP` —— 机械结构 3D 文件
- `摄像头支架/*.3mf` —— 摄像头支架 3D 打印文件

---

## 3. 硬件清单（BOM）

来源：`材料清单.docx`。

| 序号 | 名称 | 型号 / 规格 | 数量 | 备注 |
|---:|---|---|---:|---|
| 1 | 视觉核心板 | 犀牛派 A1 | 1 | 跑视觉算法（AidLux） |
| 2 | 驱动核心板 | NUCLEO-G071RB | 1 | 接收运动指令并驱动电机 |
| 3 | 电机驱动板 | 自制 V1.0（TB6612FNG） | 1 | 见 `电机驱动板原理图/` |
| 4 | 履带式电机 | 12 V | 1 对 | 淘宝采购 |
| 5 | USB 摄像头 | DCXGW10 模组，无畸变（杰锐微通） | 1 | 接犀牛派 A1 |
| 6 | 锂电池组 | 3S 11.1 V / 3 A | 1 | 给电机 + 驱动板供电 |
| 7 | Micro USB 线 | — | 1 | 犀牛派 ↔ STM32 串口 |
| 8 | 框架板 | 环氧树脂或亚克力 | 2 | 上下两层，见 `小车框架基板/` |
| 9 | M3 六棱柱 | — | 8 + 4 + 4 | 支撑上下板 |
| 10 | 摄像头支架 | 3D 打印件 | 1 套 | 见 `摄像头支架/`，可不用 |
| 11 | 扎带、M3 螺丝 | — | 若干 | 线束固定 |

---

## 4. 系统总体架构

```
┌──────────────────────────────┐         USB-UART          ┌─────────────────────────────┐
│       犀牛派 A1 (AidLux)      │   5A A5 [l_pwm][r_pwm] FF  │     STM32G071RB (Nucleo)     │
│                              │ ──────────────────────────▶ │                             │
│  USB 摄像头 (MJPG 640x480)    │                            │  USART2 115200 中断接收      │
│         │                    │ ◀────────────────────────── │         │                   │
│         ▼                    │     (回显原始帧，可选)        │         ▼                   │
│  SSDLite-MobileNetV3 (tflite)│                            │  TB6612_SetMotor()           │
│         │                    │                            │         │                   │
│         ▼                    │                            │         ▼                   │
│  SimpleTracker (IoU)          │                            │  TIM1 CH1/CH2 PWM           │
│         │                    │                            │  GPIO 方向 (PB0..PB3)        │
│         ▼                    │                            │         │                   │
│  横向 PD + 纵向 PID          │                            │         ▼                   │
│         │                    │                            │  TB6612FNG → 左右履带电机    │
│         ▼                    │                            │                             │
│  send_motor_pwm()            │                            │                             │
└──────────────────────────────┘                            └─────────────────────────────┘
```

- **能量流**：3S 11.1 V 电池 → 电机驱动板 → TB6612FNG → 两个 12 V 履带电机；同一电源域给 STM32 与犀牛派 A1 供电（实际按各板额定电压需经 DC-DC，见驱动板原理图）。
- **信息流**：摄像头帧 → A1 推理 → 解算左右轮 PWM → UART 发给 STM32 → STM32 立刻映射成 PWM + 方向引脚。

---

## 5. 机械结构

- `小车框架基板/上板.STEP` / `下板.STEP`：上下两层板，STP 格式可直接导入 SolidWorks / Fusion 360 / FreeCAD。
- `小车框架基板/上板_2004.dwg` / `下板_2004.dwg`：AutoCAD 2004 版二维出图，用于激光切割亚克力/环氧树脂板。
- `摄像头支架/`：`摄像头外壳.3mf`、`摄像头支架.3mf`、`摄像头顶盖.3mf`，Cura / PrusaSlicer 直接切片打印；也可省略，把摄像头直接绑在车架上。

层间用 M3 六立柱支撑，螺丝规格统一为 M3。

---

## 6. 硬件接线

总览图：[`接线图/系统接线图.png`](接线图/系统接线图.png)

分模块接线图：

| 模块 | 文件 |
|---|---|
| STM32 开发板 | [`接线图/STM32开发板.png`](接线图/STM32开发板.png) |
| 犀牛派 A1 | [`接线图/犀牛派.png`](接线图/犀牛派.png) |
| USB 摄像头 | [`接线图/摄像头.png`](接线图/摄像头.png) |
| 电机驱动板 | [`接线图/电机驱动板.png`](接线图/电机驱动板.png) |
| 履带电机 | [`接线图/电机.png`](接线图/电机.png) |
| 电池 | [`接线图/电池png.png`](接线图/电池png.png) |

**关键连线**：

- 犀牛派 A1 ↔ STM32：**Micro USB 线**（A1 的 USB 口 → Nucleo 的 ST-Link USB 口，虚拟串口 `ttyUSB*` / `COM*`），波特率 **115200 8N1**。
- STM32 ↔ TB6612：
  - 左电机方向：**PB0 / PB1**
  - 右电机方向：**PB2 / PB3**
  - 左电机 PWM：**TIM1_CH1**；右电机 PWM：**TIM1_CH2**（由 CubeMX 分配，见 `diy_TrackCar.ioc`）
  - TB6612 `STBY`：GPIOB 任意输出脚（代码中拉高即退出待机）
- 电池 → 驱动板 VM；驱动板 OUT1/OUT2、OUT3/OUT4 分别接左右电机。

> 实际引脚号以 `电机驱动程序/diy_TrackCar/diy_TrackCar.ioc` 与 `main.h` 中的宏定义为准（`LEFT_WHELL_IN0_Pin` 等）。

---

## 7. 电机驱动板

自制 V1.0，以 **TB6612FNG** 为核心双 H 桥：

- 原理图：[`电机驱动板原理图/电机驱动板V1.0.pdf`](电机驱动板原理图/电机驱动板V1.0.pdf)
- BOM / 坐标：[`电机驱动板原理图/电机驱动板V1.0.xlsx`](电机驱动板原理图/电机驱动板V1.0.xlsx)
- 立创EDA / 工程源文件：[`电机驱动板原理图/电机驱动板V1.0.zip`](电机驱动板原理图/电机驱动板V1.0.zip)

---

## 8. 软件架构

### 8.1 下位机：STM32G071 电机驱动

工程目录：`电机驱动程序/diy_TrackCar/`（STM32CubeIDE 工程，芯片 STM32G071RBTX）。

关键文件：

| 文件 | 作用 |
|---|---|
| `Core/Src/main.c` | 时钟、GPIO、USART2、TIM1 PWM 初始化；主循环 `USART2_Poll_Parse()` |
| `Core/Src/drv_TB6612FNG.c/.h` | TB6612 封装：`TB6612_SetMotor(l, r)`、前进/后退/原地转弯/急停等动作接口 |
| `Core/Src/drv_usart2_poll.c/.h` | USART2 中断接收 + 7 字节帧解析；收到合法帧后立刻调 `TB6612_SetMotor` |
| `diy_TrackCar.ioc` | CubeMX 配置，可重新生成外设初始化代码 |
| `STM32G071RBTX_FLASH.ld` | 链接脚本 |

关键参数（来自 `main.c`）：

- TIM1：PSC = 4，ARR = 999，APB1 时钟下 PWM 频率 ≈ 10 kHz。
- USART2：115200 8N1，**中断接收 1 字节**，在 `HAL_UART_RxCpltCallback` 里做状态机解析；主循环里只做"整帧处理"，**不在中断里做 sprintf**。
- PWM 数值范围：**-999 ~ +999**，正 = 前进，负 = 后退，0 = 刹车。

### 8.2 上位机：犀牛派 A1 视觉跟随

工程目录：`视觉控制算法程序/aid-Car/`，运行于犀牛派 A1 的 **AidLux** 环境。

| 子目录 | 功能 |
|---|---|
| `object_det/` | 主程序：USB 摄像头 → SSDLite-MobileNetV3 人体检测 → SimpleTracker → 横向 PD + 纵向 PID → 串口下发左右轮 PWM |
| `plate_spot/` | 独立 demo：基于 ONNX Runtime 的车牌检测 + 识别（PaddleOCR 风格模型），不参与小车控制 |

`object_det/` 关键文件：

| 文件 | 作用 |
|---|---|
| `run_object_det.py` | 主循环：摄像头采集、推理、跟踪、控制解算、串口下发 |
| `utils/ssd_mobilenet_utils.py` | 图像预处理、NMS、坐标反归一化 |
| `utils/SimpleTracker.py` | 基于 IoU 的简易多目标跟踪（`max_lost=3`） |
| `model_data/ssdlite_mobilenet_v3.tflite` | 量化后的检测模型（COCO 80 类） |
| `model_data/coco_classes.txt` | 类别表，类别 ID 1 = `person` |
| `requirements.txt` | 依赖清单（历史版本，见下方"复现步骤"说明） |

`plate_spot/` 关键文件：

| 文件 | 作用 |
|---|---|
| `run_plate_spot.py` | 车牌检测 + 端到端识别主程序 |
| `model/mobile_ocrv4_det.onnx` | 文本检测模型 |
| `model/mobile_ocrv4_rec.onnx` | 文本识别模型 |
| `model/ppocr_keys_v1.txt` | 识别字符表 |
| `config/simfang.ttf` | 可视化用中文字体 |

### 8.3 串口通信协议

上位机 → 下位机，**7 字节定长帧，小端打包但数值按大端字段**：

| 字节 | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| 含义 | 帧头 0 | 帧头 1 | 左 PWM 高字节 | 左 PWM 低字节 | 右 PWM 高字节 | 右 PWM 低字节 | 帧尾 |
| 值 | `0x5A` | `0xA5` | `(l>>8)&0xFF` | `l&0xFF` | `(r>>8)&0xFF` | `r&0xFF` | `0xFF` |

- `l`、`r` 为有符号 16 位整数，范围 **-999 ~ +999**。代码里用 `pwm & 0xFFFF` 做无符号化再拆字节；STM32 端接收后按 `int16_t` 还原。
- 下位机收到合法帧后，会把原始 7 字节通过同串口回显（调试用），格式为 `RX Frame: XX XX XX XX XX XX XX \r\n`。

对应代码：

- 发送：[`run_object_det.py` 中的 `send_motor_pwm()`](视觉控制算法程序/aid-Car/object_det/run_object_det.py)
- 接收：[`drv_usart2_poll.c` 中的 `HAL_UART_RxCpltCallback()`](电机驱动程序/diy_TrackCar/Core/Src/drv_usart2_poll.c)

---

## 9. 复现步骤

### 9.1 机械组装

1. 按 `小车框架基板/` 中的 DWG 切割上下两层亚克力/环氧树脂板，或直接用 STEP 3D 打印。
2. 用 M3 六立柱把上下板叠起来；上层装犀牛派 A1 + 摄像头，下层装 Nucleo 板 + 自制驱动板 + 电池。
3. 3D 打印摄像头支架，把 USB 摄像头固定在车头朝向正前方。

### 9.2 烧录 STM32 下位机

1. 安装 **STM32CubeIDE**（推荐 1.13+，配套 G0 系列 HAL 库）。
2. 导入 `电机驱动程序/diy_TrackCar/` 工程。
3. 用 ST-Link / Nucleo 板载调试器编译并下载到 NUCLEO-G071RB。
4. 上电后板载绿色 LED 应常亮；用 USB 线把 Nucleo 连到电脑，打开串口助手（115200 8N1），手动发送 `5A A5 03 E7 03 E7 FF` 应能让小车前进（左、右 PWM = +999）。

### 9.3 犀牛派 A1 上位机部署

犀牛派 A1 出厂已带 AidLux 环境。需要额外确认：

1. USB 摄像头插好后，`ls /sys/class/video4linux/` 能看到 `video0` 之类的节点（代码里 `get_cap_id()` 会自动挑第一个 USB 摄像头）。
2. 串口设备：Nucleo 虚拟串口一般在 `/dev/ttyUSB0` 或 `/dev/ttyACM0`，在 `run_object_det.py` 里打开对应设备（搜索 `serial.Serial(` 处修改）。
3. 安装 Python 依赖：
   ```bash
   pip install pyserial onnxruntime numpy
   # AidLux 自带 aidlite / aidcv / android 模块，不需要也不能用 pip 装
   ```
   > ⚠️ 仓库里 `object_det/requirements.txt` 是 AidLux 老示例自带的历史依赖（`tensorflow>=1.14`、`opencv-python==3.4.0.12` 等），**那是原示例在普通 Linux/手机环境下的依赖清单，不是本工程在犀牛派 A1 上的真实依赖**。在犀牛派 A1 上请用 AidLux 自带的 `aidlite` / `aidcv`，不要按该文件 pip 安装。

### 9.4 运行

```bash
# 在犀牛派 A1 的 AidLux 终端里
cd 视觉控制算法程序/aid-Car/object_det
python run_object_det.py
```

把人放到摄像头前方 1–3 m 内，小车应开始跟随。若发现左右轮方向接反，把 `run_object_det.py` 里的 `SWAP_WHEELS = True` 改一下，或直接交换电机两根线。

---

## 10. 控制算法说明

`run_object_det.py` 里的核心参数（已在代码里用 `====` 分隔块标出）：

**目标锁定**：

- 只取 COCO 类别 `1`（person），置信度阈值 `threshold = 0.6`。
- 用 `SimpleTracker(max_lost=3)` 做 IoU 跟踪，跨帧维持目标 ID。

**距离控制（纵向，沿摄像头光轴）**：

- 期望目标框高度 `target_height = (205 + 365) / 2 = 285 px`。
- 框高度 `< 205 px` → 人太远，前进；`> 365 px` → 人太近，后退。
- 用 PID：`KP_Z=0.35, KI_Z=0.008, KD_Z=0.05`，积分限幅 `INTEGRAL_LIMIT=500`。
- 最大前进速度 56，最大后退速度 28（归一化到 92 再 ×999/92 下发，见 `sendControlToVehicle`）。

**方向控制（横向）**：

- 误差 `error_x = 1 - 2 * foot_x / w`，正 = 目标偏左。
- 一阶低通滤波（`FILTER_ALPHA=0.45`）后做 PD：`KP_X=80, KD_X=110`，输出 `turn_adjust` 限幅在 ±14。
- 目标走到画面左右边缘（`EDGE_THRESHOLD=66 px`）时进入"边缘追出"模式，直接给固定差速，避免目标出框后丢失。
- 目标丢失最多维持 `LOST_PREDICT_DURATION=8` 帧、按最后一次转向方向继续前进，之后停车。

---

## 11. 常见问题（FAQ）

**Q1：插上 USB 线后犀牛派 A1 找不到串口？**
先在 A1 上 `ls /dev/ttyUSB* /dev/ttyACM*`。Nucleo 板载 ST-Link 一般枚举出 `ttyACM0`。如果权限不足，`sudo usermod -aG dialout $USER` 后重登，或在脚本里以 root 运行。

**Q2：小车一上电就猛冲 / 反向冲？**
检查：
1. TB6612 的 `STBY` 是否被拉高（`TB6612_Init()` 里有 `HAL_GPIO_WritePin(..., SET)`）。
2. 电机方向线是否接反——直接在 `run_object_det.py` 里把 `SWAP_WHEELS` 置 `True`，或在 STM32 端把左右 PWM 取反。
3. 电池电压是否足够（3S 满电 12.6 V，低于 10 V 建议充电）。

**Q3：检测不到人 / 帧率很低？**
- 确认摄像头在 640×480 MJPG 模式打开（代码里已 `cap.set(6, fourcc('M','J','P','G'))`）。
- 模型输入是 300×300 量化 uint8，不要随意改大。
- 光照不足时 person 置信度会掉到 0.6 以下，可临时把 `threshold` 降到 0.4。

**Q4：串口助手手动发帧小车不动？**
用逻辑分析仪或直接看 STM32 串口回显。如果完全没有 `RX Frame:` 回显，说明 USART2 没收到字节——检查 TX/RX 是否交叉、GND 是否共地。

**Q5：`import aidlite / aidcv` 报错？**
这些是 AidLux 专属模块，只能在犀牛派 A1 / AidLux 环境里 import，不能在普通电脑的 Python 里跑通。在 PC 上调试时请用 `opencv-python` 替换 `aidcv`，并把 `aidlite` 推理换成 `tflite-runtime`。

---

## 12. 仓库目录结构

```
.
├── README.md                     ← 本文件
├── 材料清单.docx                   ← BOM
├── 案例征集规范.pdf                ← 阿加犀案例征集活动规范（参考）
│
├── 小车框架基板/                   ← 机械结构
│   ├── 上板.STEP / 上板_2004.dwg
│   └── 下板.STEP / 下板_2004.dwg
│
├── 接线图/                         ← 整机与分模块接线图（PNG）
│   ├── 系统接线图.png
│   ├── STM32开发板.png / 犀牛派.png / 摄像头.png
│   ├── 电机.png / 电机驱动板.png / 电池png.png
│
├── 摄像头支架/                     ← 3D 打印件（3mf）
│   ├── 摄像头外壳.3mf
│   ├── 摄像头支架.3mf
│   └── 摄像头顶盖.3mf
│
├── 电机驱动板原理图/               ← 自制 TB6612 驱动板
│   ├── 电机驱动板V1.0.pdf
│   ├── 电机驱动板V1.0.xlsx        ← BOM / 坐标
│   └── 电机驱动板V1.0.zip          ← EDA 工程源文件
│
├── 电机驱动程序/                   ← STM32CubeIDE 工程
│   └── diy_TrackCar/
│       ├── Core/
│       │   ├── Inc/  (drv_TB6612FNG.h, drv_usart2_poll.h, main.h, ...)
│       │   └── Src/  (main.c, drv_TB6612FNG.c, drv_usart2_poll.c, ...)
│       ├── Drivers/  (STM32 HAL / CMSIS)
│       ├── diy_TrackCar.ioc
│       └── STM32G071RBTX_FLASH.ld
│
└── 视觉控制算法程序/               ← 犀牛派 A1 / AidLux Python 工程
    └── aid-Car/
        ├── examples.json           ← AidLux 应用注册表
        ├── object_det/             ← 主程序：人体检测 + 跟随
        │   ├── run_object_det.py
        │   ├── utils/  (SimpleTracker.py, ssd_mobilenet_utils.py, ...)
        │   └── model_data/  (ssdlite_mobilenet_v3.tflite, coco_classes.txt, ...)
        └── plate_spot/             ← 独立 demo：车牌识别（ONNX）
            ├── run_plate_spot.py
            ├── model/  (mobile_ocrv4_*.onnx, ppocr_keys_v1.txt)
            └── img/  (测试图)
```

> 备注：`电机驱动程序/diy_TrackCar/Debug/` 下的 `.o / .elf / .map / .list / .d` 等为 STM32CubeIDE 编译产物，`视觉控制算法程序/.../__pycache__/` 为 Python 字节码缓存，当前已随首版入库；后续建议加入 `.gitignore` 并从索引中移除。

---

## 13. 开源协议

本项目采用 **MIT License**。基于阿加犀《开发者案例征集规范》要求，原作者同意将本案例开源，允许商用、修改与再分发，但需保留版权与许可声明。

> 根目录应同步放置一份 `LICENSE` 文件（MIT 全文）。若缺失，请从 <https://opensource.org/licenses/MIT> 拷贝一份并填入：
>
> ```
> MIT License
>
> Copyright (c) 2026 h27059
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> ...
> ```

---

## 14. 致谢与生态标识

- 视觉核心板由 **阿加犀（Aglatech）犀牛派生态**提供，运行于 **AidLux** 系统。
- 本案例属于 `#犀牛派开发者案例` `#阿加犀生态` 征集活动范畴。
- 检测模型 SSDLite-MobileNetV3、车牌识别模型（PaddleOCR 风格）权重版权归原作者所有。
- STM32 HAL / CMSIS 由 STMicroelectronics 提供（见 `Drivers/*/LICENSE.txt`）。

---

*最后更新：2026-09-28。*
