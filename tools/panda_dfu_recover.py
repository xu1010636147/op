#!/usr/bin/env python3
"""panda 救砖脚本（方案 B：硬件强制进入 ST ROM DFU）。

适用现象：panda 的 LED 熄灭 / `lsusb` 看不到设备 / bootstub 已损坏，
即 `panda/board/recover.py` 和 pandad 的自动恢复都无从下手时。

时序（依据 panda 源码）：
  * 固件侧的 USB 描述符是固定的 USB_VID/USB_PID（panda/board/config.h），
    所以 0483:df11 只可能来自 STM32 出厂 ROM bootloader。
  * 写入 bootstub 并跳转后，bootstub 校验 app 失败（app 区已被擦除）
    -> soft flasher，以 PID 0xDDEE 枚举，此时 bootstub=True（panda/python/__init__.py:326）。
  * 只有到了这一步，才能用 panda 协议写 app。

用法：
  .venv/bin/python tools/panda_dfu_recover.py            # 交互确认后恢复
  .venv/bin/python tools/panda_dfu_recover.py --yes      # 免确认
  .venv/bin/python tools/panda_dfu_recover.py --build    # 先 scons 重新编译

需要 root 吗？不需要。99-panda.rules 已对 0483:df11 开 MODE=0666。
"""

import argparse
import os
import subprocess
import sys
import time

BASEDIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, BASEDIR)

try:
  from panda import Panda, PandaDFU, FW_PATH
except ImportError as e:
  sys.stderr.write(
    f"无法导入 panda 库（{e}）。请用仓库自带虚拟环境运行：\n"
    f"  {os.path.join(BASEDIR, '.venv/bin/python')} {os.path.realpath(__file__)}\n"
  )
  raise SystemExit(1)


DFU_VID_PID = "0483:df11"


def log(msg: str = "") -> None:
  print(msg, flush=True)


def wait_for_dfu(timeout: float) -> str:
  """轮询等待 ST ROM DFU 设备出现，返回其 DFU 序列号。"""
  t_start = time.monotonic()
  next_report = t_start

  while True:
    serials = PandaDFU.usb_list()
    if len(serials) == 1:
      return serials[0]
    if len(serials) > 1:
      raise SystemExit(f"发现 {len(serials)} 个 DFU 设备 {serials}，请只保留一块 panda 后重跑")

    now = time.monotonic()
    if now - t_start > timeout:
      raise SystemExit(f"等待 DFU 设备超时（{timeout:.0f}s），未出现 {DFU_VID_PID}")
    if now >= next_report:
      log(f"  ... 等待 {DFU_VID_PID}，剩余 {timeout - (now - t_start):.0f}s")
      next_report = now + 5.0
    time.sleep(0.3)


def check_firmware(cfg) -> None:
  """确认待写入的固件产物存在且大小合理。"""
  for fn, addr in ((cfg.bootstub_fn, cfg.bootstub_address), (cfg.app_fn, cfg.app_address)):
    path = os.path.join(FW_PATH, fn)
    if not os.path.isfile(path) or os.path.getsize(path) < 4096:
      raise SystemExit(f"固件缺失或过小：{path}\n请先编译：scons -C {BASEDIR}/panda -j$(nproc) {BASEDIR}/panda/board")
    log(f"  {fn:24s} {os.path.getsize(path):>7d} B  ->  {addr:#010x}")


def pick_single_panda(timeout: float) -> Panda:
  """等待且仅接受一个 panda，避免误刷到别的设备。"""
  if not Panda.wait_for_panda(None, timeout):
    if PandaDFU.usb_list():
      raise SystemExit("设备仍停留在 DFU：bootstub 未成功启动，请重跑本脚本")
    raise SystemExit(f"等待 panda 出现超时（{timeout:.0f}s）")

  serials = Panda.list()
  if len(serials) != 1:
    raise SystemExit(f"期望恰好 1 个 panda，实际 {serials}，请拔掉多余的设备后重跑")
  return Panda(serials[0])


def main() -> int:
  parser = argparse.ArgumentParser(description="panda 救砖（方案 B：硬件进入 ST ROM DFU）")
  parser.add_argument("--yes", action="store_true", help="跳过擦除前的交互确认")
  parser.add_argument("--build", action="store_true", help="先重新编译 panda 固件")
  parser.add_argument("--timeout", type=float, default=300.0, help="等待设备出现的超时秒数（默认 300）")
  args = parser.parse_args()

  log("== panda 救砖（方案 B）==")
  log()
  log("硬件步骤（本脚本会在此等待）：")
  log("  1) 拔掉 panda 的 USB")
  log("  2) 按住 BOOT0（板载按键 / panda paw），保持按住")
  log(f"  3) 插入 USB，确认 `lsusb | grep 0483:df11` 出现后再松手")
  log()

  if args.build:
    log(">>> 重新编译 panda 固件 ...")
    subprocess.check_call(f"scons -C {BASEDIR}/panda -j$(nproc) {BASEDIR}/panda/board", shell=True)

  # bootstub 完好时不该走这条路，直接劝退，避免无谓的整片擦除
  if not PandaDFU.usb_list():
    try:
      alive = Panda.list()
    except Exception:
      alive = []
    if alive:
      log(f"当前已有 {len(alive)} 个可正常枚举的 panda {alive} —— bootstub 未损坏。")
      log("用软件路径即可，无需硬件进 DFU：")
      log(f"  {os.path.join(BASEDIR, '.venv/bin/python')} {BASEDIR}/panda/board/recover.py")
      return 2

  log(">>> 等待 ST ROM DFU 设备 ...")
  dfu_serial = wait_for_dfu(args.timeout)
  log(f"发现 DFU 设备：{dfu_serial}")

  dfu = PandaDFU(dfu_serial)
  mcu = dfu.get_mcu_type()
  cfg = mcu.config
  log(f"MCU 识别：{mcu.name}  (app @ {cfg.app_address:#010x}, bootstub @ {cfg.bootstub_address:#010x})")
  log()
  log("待写入固件：")
  check_firmware(cfg)

  if not args.yes:
    log()
    log("⚠️  危险操作：将擦除该 MCU 的全部 flash sector，再写入上面的 bootstub 和 app。")
    log("    擦除过程中掉电/拔线，可能使 panda 无法再通过 USB 恢复（需 SPI 或 SWD）。")
    try:
      answer = input("确认继续？输入 yes 继续：")
    except EOFError:
      dfu.close()
      log("非交互环境且未指定 --yes，已取消。")
      return 1
    if answer.strip().lower() not in ("y", "yes"):
      dfu.close()
      log("已取消。")
      return 1

  log()
  log(">>> 擦除全部 sector 并写入 bootstub ...")
  dfu.recover()  # clear_status -> erase all sectors -> program bootstub -> jump
  dfu.close()
  log("    完成，bootstub 已接管")

  log(">>> 等待 panda 以 bootstub 模式枚举 ...")
  panda = pick_single_panda(args.timeout)
  if not panda.bootstub:
    panda.close()
    raise SystemExit("目标未处于 bootstub 模式，中止（避免误刷正在运行的 panda）")

  log(f">>> 写入 app 固件（{cfg.app_fn}）...")
  panda.flash()  # 内部以 0xd8 复位，随后 bootstub 校验通过并跳转 app
  log("    完成")

  expected = Panda.get_signature_from_firmware(os.path.join(FW_PATH, cfg.app_fn))
  panda.reconnect()  # 重新读取 bootstub 标志与版本

  log()
  log("== 恢复结果 ==")
  log(f"  序列号 : {panda.get_usb_serial()}")
  log(f"  MCU    : {mcu.name}")
  log(f"  版本   : {panda.get_version()}")

  if panda.bootstub:
    panda.close()
    raise SystemExit("panda 仍停留在 bootstub：app 未能启动，通常是固件签名不被接受")

  signature = panda.get_signature()
  log(f"  签名   : {signature.hex()[:16]}  (期望 {expected.hex()[:16]})")
  panda.close()

  if signature != expected:
    log("⚠️  签名不一致：pandad 下次启动会重新刷写。若为自编译固件，请确认已正确签名。")
    return 1

  log("  ✅ 恢复成功")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
