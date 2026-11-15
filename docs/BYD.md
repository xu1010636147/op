# 比亚迪（BYD）车型适配说明

本分支的 BYD 支持以 mypilot / dp-BYD 的 27 平台模块为基准（`C2+DBC+副本/高阶Python源码_v2`），
移植到当前 CarrotPilot 的 opendbc API，并统一为单份 DBC。

## 1. 目录与数据

| 内容 | 位置 |
|---|---|
| 车模块 | `opendbc_repo/opendbc/car/byd/` |
| 主 DBC（19 条报文，全部平台共用） | `opendbc_repo/opendbc/dbc/byd_generic_pt.dbc` |
| 唐 DM 毫米波雷达 DBC（4 条报文） | `opendbc_repo/opendbc/dbc/byd_tangdm_radar.dbc` |
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
.venv/bin/python -m ruff check opendbc_repo/opendbc/car/byd
```

另有一套合成 CAN 冒烟：对 27 个平台各灌入 `byd_generic_pt.dbc` 的全部报文并跑
`CarState.update` + `CarController.update`，用于发现 DBC 信号名/位定义缺失。

## 7. 已知限制 / 后续

1. 角度平台（含 Seal 23）的 panda safety 未完成：需要按 `CID_TCDUBH` 的真实位定义重写
   `safety_byd.h` 的 angle 检查，并在台架验证后再打开。
2. `FwQueryConfig` 的 UDS 请求沿用了 dp 的原始字节，未在实车核对；当前车型识别完全依赖 CAN 指纹。
3. `byd_generic_pt.dbc` 的 checksum 未注册到 `opendbc/can/dbc.py`（与 dp 行为一致），
   因此 parser 不做 BYD 校验和校验；若后续要校验，需同时处理 `byd_tangdm_radar` 的不同校验方式。
4. dp 的动态参数（BSD 类型、速度修正、角度速率）尚未加进 `common/params_keys.h`，UI 不可调。
