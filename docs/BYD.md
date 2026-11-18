# 比亚迪（BYD）车型适配说明

本分支的 BYD 支持以 mypilot / dp-BYD 的 27 平台模块为基准（`C2+DBC+副本/高阶Python源码_v2`），
移植到当前 CarrotPilot 的 opendbc API，并统一为单份 DBC。

## 1. 目录与数据

| 内容 | 位置 |
|---|---|
| 车模块 | `opendbc_repo/opendbc/car/byd/` |
| 主 DBC（19 条报文，全部平台共用） | `opendbc_repo/opendbc/dbc/byd_generic_pt.dbc` |
| 唐 DM 毫米波雷达 DBC（4 条报文） | `opendbc_repo/opendbc/dbc/byd_tangdm_radar.dbc` |
| ADC 平台化 MRR DBC（49 条 CAN FD 报文） | `opendbc_repo/opendbc/dbc/byd_mrr_fd.dbc` |
| panda safety | `opendbc_repo/opendbc/safety/safety/safety_byd.h` |
| 扭矩参数 | `opendbc_repo/opendbc/car/torque_data/{params,override}.toml` |

`byd_generic_pt.dbc` 是原 `byd_han_dmev_2020.dbc` 的 dp 命名版本：报文 ID 与位定义一致，
只是信号名改为混淆名（`sig_xxxxxx`）。旧的 `byd_han_dmev_2020.dbc` 已删除。

## 2. 平台覆盖（27 个）

| 平台 | 转向控制 | panda safety |
|---|---|---|
| HAN_DM_20 / HAN_EV_20 / HAN_DMI_22 / HAN_DMI_25 / HAN_DMI_22R | 扭矩 | ✅ |
| TANG_DM / TANG_DMI_21 / TANG_DMI_24 / TANG_DMP_22 / TANG_DMP_23 | 扭矩 | ✅ |
| SONG_PLUS_DMI_21/22、SONG_PRO_DMI_22、QIN_PLUS_DMI_23、QIN_PRO、YUAN_PLUS_DMI_22 | 扭矩 | ✅ |
| HAN_DMI_22J、SONG_L_DMI_24、QIN_L_DMI、SEAL_06_DMI、SEAL_EV_22、SEAL_07_DMI、FRIGATE_07_DMI、TENGSHI_D9_22/22R/24、SEAL_23 | 角度 | ⚠️ 只读（dashcamOnly） |

扭矩平台沿用 `safety_byd.h` 的 HAN/TANG 协议：`0x316` 的 `LKAS_Output` 在 bit16、
`0x32D` 的 `AccState` 在 bit19、车速/踏板/EPS 状态报文与 safety 的 RX 检查逐位对齐。

横向速率取 `STEER_DELTA_UP/DOWN = 7/9`（`STEER_MAX=300`、50 Hz 下发），满足
`test_lateral_limits.py` 的 ISO 11270 jerk 约束；dp 原值 17/17 在该测试下会超限，实车可按
转向手感再调。

角度平台暂不启用控制：`byd_generic_pt.dbc` 里 `CID_TCDUBH`(0x1E2) 的角度在 bit24（0.1°/count），
而 `safety_byd.h` 现有 SEAL 角度检查按 bit16（1°/count）解析，两者不一致。在补齐并台架验证
safety 之前，这些平台只做状态读取，避免下发未经验证的角度指令。

## 3. 指纹与手动选择

- 19 个平台带 CAN 指纹。
- `BYD_TANG_DMI_21/_24/DMP_22/DMP_23` 的 CAN 指纹完全相同，只在 `BYD_TANG_DMI_21` 保留，
  其余三款需要手动指定（见下）。
- 另有 8 个平台源码中没有指纹数据（HAN_DMI_25、HAN_DMI_22R、HAN_DMI_22J、SONG_L_DMI_24、
  QIN_PRO、SEAL_EV_22、SEAL_07_DMI、TENGSHI_D9_22R）。

手动指定平台（`car_helpers` 支持 `FINGERPRINT` 环境变量）：

```bash
FINGERPRINT='BYD_TANG_DMI_24' ./launch_openpilot.sh
```

## 4. 参数

已接入 CarrotPilot 的参数：`BydLatUseSiglin`、`BydModifiedStockLong`、`AutoGasSyncSpeed`、
`SpeedFromPCM`、`EnableRadarTracks`。

dragonpilot 独有、本分支未注册的参数（`BydBsdType2`、`EnableExtRadar`、`SpeedCorrect30/60/90/120`、
`LateralAngle*`）通过 `byd/bydparams.py` 读取，缺省时回退到安全默认值。

## 5. 扭矩参数（上车前必须标定）

`params.toml` 里 16 个扭矩平台目前是占位值 `[1.0, 2.5, 0.15]`，与之前仓库里的 4 条 BYD 记录一致。
这些值只保证接口能构造、静态测试能跑，**不等于实车可用**。上车前按
`torque_data/README.md` 用 `torqued.py` 重新标定 `LAT_ACCEL_FACTOR` 与 `FRICTION`。

## 6. 静态验证

```bash
.venv/bin/python -m pytest opendbc_repo/opendbc/car/tests/test_car_interfaces.py -k byd -q
.venv/bin/python -m pytest opendbc_repo/opendbc/car/tests/test_can_fingerprint.py -k BYD -q
.venv/bin/python -m pytest opendbc_repo/opendbc/car/tests/test_platform_configs.py -q
.venv/bin/python -m pytest opendbc_repo/opendbc/car/tests/test_fw_fingerprint.py -k byd -q
.venv/bin/python -m pytest opendbc_repo/opendbc/car/byd/tests -q
.venv/bin/python -m ruff check opendbc_repo/opendbc/car/byd
```

另有一套合成 CAN 冒烟：对 27 个平台各灌入 `byd_generic_pt.dbc` 的全部报文并跑
`CarState.update` + `CarController.update`，用于发现 DBC 信号名/位定义缺失。

## 7. 毫米波雷达（前向）

本模块支持三套互斥的前向雷达数据源，`RadarInterface` 按参数选择：

| 数据源 | 报文 | DBC | 内容 | 开关 |
|---|---|---|---|---|
| ADC 平台化 MRR | `0x640~0x670`（CAN FD 64B） | `byd_mrr_fd.dbc` | 40 个跟踪目标 + 200 点云 + 道路模型 | `BydMrrFd=1`（或 `BYD_RADAR_MODE=ars`） |
| 唐 DM 雷达 | `0x351~0x354`（经典 CAN） | `byd_tangdm_radar.dbc` | 4 个跟踪目标 | `EnableRadarTracks>=1` 且车型在 `RADAR_CAR` |
| 车控网目标 | `CID_NCBHOL`（0x374） | `byd_generic_pt.dbc` | 左/中/右 3 个目标 | `EnableRadarTracks>=1` 且车型在 `PT_RADAR_CAR`（含 `TANG_DMI_21`） |

`BydMrrFd` 打开时会强制 `CarParams.radarUnavailable=False`，并把 `RadarInterface` 的全部工作交给
`opendbc/car/byd/radar_ars.py`（见该文件头部的信号映射表）。它默认关闭，因为：

1. `0x640~0x670` 来自 SA6/IDCC-B10 的 ADC 平台化矩阵（V2.8.0），**唐 DM-i 是否广播这套报文未验证**；
2. 这些帧是 CAN FD 64 字节，经典 CAN 通道上收不到。

上车前先跑探测脚本，确认实际存在的报文：

```bash
python3 selfdrive/debug/byd_radar_probe.py            # 实车
python3 selfdrive/debug/byd_radar_probe.py --route <route>
```

判读：

- `0x640~0x670` 存在且 DLC=32/64 → 置 `BydMrrFd=1` 即可；
- `0x640~0x670` 存在但 DLC 全是 8 → 该通道没按 CAN FD 收，先查通道/波特率；
- 只有 `0x351~0x354` → 是唐 DM 那套，走 `RADAR_CAR` 路径，不是本适配；
- 都没有 → 该车没有可读的前向雷达数据，`radarUnavailable` 保持 True。

其他可调项（`radar_ars.py` 读环境变量，便于台架）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `BYD_RADAR_BUS` | `1` | 雷达所在 CAN 通道（`CanBus.MRR`） |
| `BYD_RADAR_PROB_MIN` | `50` | 目标存在概率门限（%） |
| `BYD_RADAR_LAT_MAX` | `5.0` | 横向距离门限（m） |
| `BYD_RADAR_LONG_OFFSET` | `0.0` | dRel 补偿（雷达安装点与车头差值，m） |
| `BYD_RADAR_VREL_SIGN` | `1` | vRel 符号；实车若反向置 `-1` |
| `BYD_RADAR_CLUSTERS` | 关 | 把点云也当 RadarPoint 输出（调试，会产生大量瞬时 track） |
| `BYD_RADAR_LOG` | 无 | 每个雷达周期写一行摘要，便于路试分析 |

注意 `radar_ars` 的 `vRel` 符号约定来自 DBC 注释（“对自车纵向相对速度”），未在实车核对；
第一次路试请同时看 `BYD_RADAR_LOG` 里的 `near=` 与车机上的前车距离是否一致。

### 7.1 唐 DM-i 走 `CID_NCBHOL`（0x374）

台架实测：唐 DM-i 的前向目标只出现在 `CID_NCBHOL`，没有 `0x640~0x670`、也没有 `0x351~0x354`。
`CID_NCBHOL` 每帧只给一个"当前选中目标"（槽位 1/2/3 + 距离 + 横向偏移 + 有效位），**没有相对速度**，
所以 `radar_interface._update_pt` 用最近 5 帧距离做最小二乘拟合估计 `vRel`（斜率），再算
`vLead = vRel + v_ego`；给 NaN 或 0 会让 radard 把前车当静止车。拟合窗口不足 3 帧时 `vRel=0`。

`TANG_DMI_21` 已加入 `PT_RADAR_CAR`，效果：

- `CarParams.radarUnavailable=False`，card 开始发布 `liveTracks`（RadarData）；
- `CarState` 解析 `CID_NCBHOL` 并回填 `mrr_leading_dist`（`BydModifiedStockLong` 的 MPC ACC 用）；
- 有前车时 `liveTracks.points` 会出现 `trackId=2` 的点，`dRel/yRel/vRel/vLead` 来自上面的估计。

台架静止、前方无目标时 `valid=0`，点数应为 0、`liveTracks.errors` 全 False，这正是验证过的状态。

**未接通**：MRR FD（`0x640~0x670`）数据还没有回填 `CarState.mrr_leading_dist`（雷达接口在 carstate
之后运行）；若要让原车 ACC 也用这套目标，需要通过 Param 或 socket 把最近目标距离传给 carstate/controller。

## 8. 已知限制 / 后续

1. 角度平台（含 Seal 23）的 panda safety 未完成：需要按 `CID_TCDUBH` 的真实位定义重写
   `safety_byd.h` 的 angle 检查，并在台架验证后再打开。
2. `FwQueryConfig` 的 UDS 请求沿用了 dp 的原始字节，未在实车核对；当前车型识别完全依赖 CAN 指纹。
3. `byd_generic_pt.dbc` 的 checksum 未注册到 `opendbc/can/dbc.py`（与 dp 行为一致），
   因此 parser 不做 BYD 校验和校验；若后续要校验，需同时处理 `byd_tangdm_radar` 的不同校验方式。
4. dp 的动态参数（BSD 类型、速度修正、角度速率）尚未加进 `common/params_keys.h`，UI 不可调。
5. 唐 DM 雷达 `RelSpeed_tang` 的 DBC 单位写的是 `km/s`（应为 `km/h`），而
   `radar_interface._update_tangdm` 直接把原始值当 m/s 用；实车需要对照车机前车速度确认单位，
   若差 3.6 倍则乘 `CV.KPH_TO_MS`。0x351 原先的信号名笔误（`RelSpeed _tang`）已修正。
6. MRR FD 与 `BydModifiedStockLong` 未打通（见第 7 节末尾）。

## 9. 故障排查：启动 openpilot 后仪表盘报“巡航系统故障”

### 9.1 现象与根因

- 现象：openpilot 一启动（panda 进入 BYD safety、继电器打开、开始按 `byd_fwd_hook` 转发），
  还没接入 ACC，仪表盘就报“巡航错误 / 巡航系统故障”。
- 根因：前视相机（MPC，接在 bus 2）做 LKAS/ACC 状态机时需要 EPS 的 `ACC_EPS_STATE`
  （扭矩平台 `0x318`、Seal `0x1FC`：CruiseActivated / MainTorque / 转角）。早期移植版本在
  `byd_fwd_hook` 里把这两条从“车端 → 相机侧”也拦掉了，而 openpilot 并没有发送对应的伪造报文
  （`bydcan.create_fake_318` / `create_fake_1FC` 从未被调用），相机在 1s 内等不到 EPS 反馈就会置
  DTC，仪表盘随之报巡航故障。dp 原版 safety 的 `is_eps` 恒为 `false`，正是为了始终放行这条。
- 修复（2026-11-18）：`opendbc/safety/safety/safety_byd.h::byd_fwd_hook` 车端 → 相机侧全部放行，
  只在相机 → 车端拦 openpilot 要接管的 `0x316/0x1E2/0x32E`。

### 9.2 改完必须重编 + 刷写 panda

safety 是编译进 panda 固件的，只改 Python 不生效：

```bash
cd panda && PATH=/data/openpilot/.venv/bin:$PATH scons -j$(nproc)
# 生成 panda/board/obj/panda.bin.signed；pandad 启动时按签名不匹配自动刷写
```

签名脚本的 shebang 是 `#!/usr/bin/env python3`，必须把 `.venv/bin` 放到 PATH 前面，
否则会因系统 Python 缺 `Crypto`（pycryptodome）而签名失败。也可以手动刷：

```bash
python3 -c "from panda import Panda; Panda().flash()"
```

### 9.3 验证

1. 单元级（不接车）：编译 `opendbc/safety/tests/libsafety` 后调用 `set_safety_hooks(35, 0)`
   （`SAFETY_BYD`），`safety_fwd_hook(0, 0x318)` 应为 `2`（转发到 MPC），
   `safety_fwd_hook(2, 0x32E/0x316/0x1E2)` 应为 `-1`（拦截）。
2. **2026-11-18 实车已验证**：修复后启动 openpilot，bus 130 上 `0x318` ≈50Hz、
   `0x316/0x32E` 正常下发，仪表盘不再报巡航故障（修复前同样场景必报）。
   实车/台架复查方法：openpilot 运行后 `panda.can_recv()` 应能在 bus `130`（`0x80|2`，panda 发往相机侧
   的回环）看到 `0x318` 50Hz；修复前 bus 130 上完全没有 `0x318`。也可以查新路由 rlog 的 `can`。
3. 仍会在 bus `192`/`194` 看到 `0x32D`/`0x3B0` 被拦（`safetyTxBlocked` 每分钟数千）。
   这是 dp 的既有行为：相机自己的 `0x32D` 会正常转发到车端，`0x3B0` 也会转发到相机侧，
   **不要**把这两条加进 TX 白名单，否则同一 ID 会出现两个发送源、counter 互相打架。

### 9.4 相关：counter 未对齐

`0x316`/`0x32E` 的 rolling counter 从 0 开始，没有用相机当前的 counter 对齐（高阶 dp 源码
同样如此，`first_start` 是死变量）。目前实车没有因 counter 跳变报错；如果以后出现“接入瞬间
就报故障”，可以参考 `比亚迪和吉利的部分源码/byd源码/carcontroller.py::_initialize_counters`
用 `CS.acc_mpc_state_counter` / `CS.acc_cmd_counter` 对齐。
