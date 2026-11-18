# 项目级记忆（AGENTS.md）

本文件由 DeepSeek Harness 作为项目级指令随会话自动加载，记录本项目（/data/openpilot，CarrotPilot 分支）需要长期记住的结论。

## 车辆 / LKAS

### LKAS 报错只能重启车辆才能消除

- 现象：车辆出现 LKAS（车道保持辅助）故障/受限提示。
- 结论：**软件层面无法清除**，只有重启车辆（整车下电再上电）才能消除该状态。
- 处理约定：遇到 LKAS 报错时，不要反复尝试重启 openpilot 进程、清参数或重新点火 CAN 来"复位"；直接按"重启车辆"处理并向用户说明。

## 车辆 / 比亚迪（BYD）

- 车模块：`opendbc_repo/opendbc/car/byd/`，27 个平台，统一使用 `byd_generic_pt.dbc`（唐 DM 雷达用 `byd_tangdm_radar.dbc`）。详见 `docs/BYD.md`。
- 转向控制：HAN/TANG/SONG/QIN/YUAN 等 16 个平台走扭矩控制；HAN_22J、SONG_L、QIN_L、SEAL 系列、FRIGATE、TENGSHI 等 11 个平台走角度控制。
- **panda safety 只覆盖扭矩协议平台**（`safetyParam = HAN_TANG_DMEV`）。角度平台在 `interface.py` 里全部 `dashcamOnly=True`，因为 `CID_TCDUBH`(0x1E2) 的角度位定义与 `safety_byd.h` 现有 SEAL 角度检查不一致；未补齐并台架验证前不要放开。
- 唐 DM-i 21/24、唐 DMP 22/23 的 CAN 指纹完全相同，只保留在 `BYD_TANG_DMI_21` 上；其余车型需用 `FINGERPRINT='BYD_...'` 环境变量手动指定。另有 8 个平台源码无指纹。
- 扭矩参数 `params.toml` 中的 BYD 值目前是占位 `[1.0, 2.5, 0.15]`，只保证接口构造与静态测试通过，**实车必须重新标定** `LAT_ACCEL_FACTOR`/`FRICTION`。
- dp 独有参数（`BydBsdType2`、`EnableExtRadar`、`SpeedCorrect*`、`LateralAngle*`）未注册进 `params_keys.h`，由 `byd/bydparams.py` 容错读取默认值。
- 前向雷达：三套数据源互斥，`BydMrrFd`（新注册到 `params_keys.h`）走 ADC 平台化 MRR 的 CAN FD 目标列表 `0x640~0x670`；唐 DM 雷达 `0x351~0x354` 走 `RADAR_CAR`；车控网 `CID_NCBHOL`(0x374) 走 `PT_RADAR_CAR`。`0x640~0x670` 是 SA6/IDCC-B10 矩阵，**唐 DM-i 是否广播、是否 CAN FD 均未实车验证**，上车前先跑 `selfdrive/debug/byd_radar_probe.py`。
- 台架实测：唐 DM-i 总线上只有 `0x374`（没有 0x640~0x670 / 0x351~0x354），`TANG_DMI_21` 已加入 `PT_RADAR_CAR`；`CID_NCBHOL` 不带速度，`radar_interface._update_pt` 用距离历史最小二乘拟合 `vRel` 并算 `vLead`（别再写回 NaN/0）。
- 台架 `BYD_TANG_DMI_21` 自动指纹失败（比指纹多 0x25C/0x336/0x33F/0x38F/0x405/0x4BB/0x4CA/0x4FE），启动需 `FINGERPRINT=BYD_TANG_DMI_21`；`launch_pc.sh` 在 tmux 里起不来 UI（X 授权），需在桌面终端启动。

### 仪表盘报“巡航系统故障/巡航错误”（2026-11-18 已修）

- 现象：启动 openpilot、panda 进入 BYD safety 后，还没接入 ACC，仪表盘就报巡航故障。
- 根因：`safety_byd.h::byd_fwd_hook` 把车端 → 相机侧的 `0x318`(扭矩平台)/`0x1FC`(Seal)
  （ACC_EPS_STATE，EPS 状态/主扭矩/转角）也拦掉了；前视相机（MPC）收不到 EPS 反馈会在 1s 内置 DTC，
  仪表盘随之报巡航故障。dp 原版 safety 的 `is_eps` 恒为 false，一直是放行的；`create_fake_318`
  在 dp 和本分支都从未被调用，所以不能靠“造一条假 0x318”补。
- 修复：车端 → 相机侧全部放行，只拦相机 → 车端的 `0x316/0x1E2/0x32E`。
- **safety 是烧进 panda 固件的**：改完必须 `cd panda && PATH=/data/openpilot/.venv/bin:$PATH scons -j$(nproc)`
  （签名脚本走 `#!/usr/bin/env python3`，PATH 不前置 venv 会缺 Crypto 签名失败），然后
  `panda.flash()`；pandad 启动时也会按固件签名不匹配自动刷。
- 验证：openpilot 运行时 `panda.can_recv()` 的 bus `130`(0x80|2) 上应出现 `0x318` 50Hz（修复前没有）。
  bus `192`/`194` 仍拦 `0x32D`/`0x3B0` 是 dp 既有行为，相机自己的 0x32D/0x3B0 本来就会转发，
  **不要**把它们加进 TX 白名单。
- **2026-11-18 实车确认**：刷完新固件启动 openpilot，仪表盘不再报巡航错误；bus 130 上 `0x318` ≈50Hz。
- 另：`0x316/0x32E` 的 counter 从 0 开始、未与相机对齐（高阶 dp 源码也一样），目前未报错；
  若出现“接入瞬间报故障”，参考 `byd源码/carcontroller.py::_initialize_counters` 对齐。
