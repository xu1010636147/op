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
