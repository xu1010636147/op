#!/usr/bin/env python3
"""离线分析 rlog 里的 ACC_MPC_STATE(790) 帧。
用法:
  python3 analyze_rlog_790.py                        # 分析最新一段录制
  python3 analyze_rlog_790.py <rlog.zst路径>         # 分析指定路径
输出:
  - 790 在 bus 2 上的总帧数
  - 每个 bit 的翻转次数(次数高=活跃,0=静态)
  - payload 变化时的 diff 日志
  - 初始/最终 payload 的 DBC 解码(LKAS_Config 等关键位)
"""
import sys, os, glob

sys.path.insert(0, '/data/openpilot')
from openpilot.tools.lib.logreader import LogReader

def find_latest():
    paths = sorted(glob.glob('/home/xuqi/.comma/media/0/realdata/*--*--0/rlog.zst'),
                   key=os.path.getmtime, reverse=True)
    return paths[0] if paths else None

def decode_acc_mpc_state(payload: bytes):
    """按 byd_han_dmev_2020.dbc 解码 ACC_MPC_STATE 的关键位。"""
    if len(payload) < 8:
        return {}
    b = payload
    # Intel little-endian: bit 位置 = byte*8 + bit_in_byte
    def get_bits(start_bit, length):
        v = 0
        for i in range(length):
            bit = start_bit + i
            if b[bit // 8] & (1 << (bit % 8)):
                v |= (1 << i)
        return v
    return {
        'LKAS_Config':               get_bits(6, 2),   # 0=DISABLE,1=ALARM,2=LKA,3=ALARM_AND_LKA
        'ReqHandsOnSteeringWheel':   get_bits(10, 1),
        'MPC_State':                 get_bits(11, 4),
        'LKAS_ReqPrepare':           get_bits(27, 1),
        'LKAS_Active':               get_bits(28, 1),
        'LKAS_State':                get_bits(36, 4),
        'Counter':                   get_bits(52, 4),
        'CheckSum':                  get_bits(56, 8),
    }

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else find_latest()
    if not path or not os.path.exists(path):
        print(f"找不到 rlog: {path}")
        return
    print(f"[rlog] {path}\n")

    bit_flip = [0] * 64   # 64 bit 累计翻转次数
    frames_bus2 = 0
    frames_bus0 = 0
    last_b2 = None
    first_b2 = None
    diff_log = []

    for msg in LogReader(path):
        if msg.which() != 'can':
            continue
        for can in msg.can:
            if can.address != 790:
                continue
            if can.src == 0:
                frames_bus0 += 1
                continue
            if can.src != 2:
                continue
            frames_bus2 += 1
            dat = bytes(can.dat)
            if first_b2 is None:
                first_b2 = dat
                last_b2 = dat
                diff_log.append(('INIT', 0, dat, {}))
            elif dat != last_b2:
                # 计算 bit 翻转
                flipped = []
                for byte_i in range(min(8, len(dat))):
                    x = dat[byte_i] ^ last_b2[byte_i]
                    for bit_i in range(8):
                        if x & (1 << bit_i):
                            bit_flip[byte_i * 8 + bit_i] += 1
                            flipped.append((byte_i, bit_i))
                diff_log.append(('DIFF', frames_bus2, dat, flipped))
                last_b2 = dat

    print(f"[统计] bus2 上 790 总帧数: {frames_bus2}, bus0 上 790 总帧数: {frames_bus0}")
    if frames_bus2 == 0:
        print("bus 2 上没抓到 790，rlog 可能不完整"); return

    print(f"\n[起始帧] {first_b2.hex(' ')}")
    print(f"         DBC解码: {decode_acc_mpc_state(first_b2)}")
    print(f"[终止帧] {last_b2.hex(' ')}")
    print(f"         DBC解码: {decode_acc_mpc_state(last_b2)}")

    print(f"\n[bit翻转统计] (bit位置 → 翻转次数)")
    active = [(i, c) for i, c in enumerate(bit_flip) if c > 0]
    for bit_pos, cnt in active:
        byte_i, bit_i = bit_pos // 8, bit_pos % 8
        note = ""
        if bit_pos == 6 or bit_pos == 7:
            note = "  ← DBC说这是 LKAS_Config"
        elif 10 <= bit_pos <= 10:
            note = "  ← ReqHandsOnSteeringWheel"
        elif 11 <= bit_pos <= 14:
            note = "  ← MPC_State"
        elif 27 == bit_pos:
            note = "  ← LKAS_ReqPrepare"
        elif 28 == bit_pos:
            note = "  ← LKAS_Active"
        elif 36 <= bit_pos <= 39:
            note = "  ← LKAS_State"
        elif 52 <= bit_pos <= 55:
            note = "  ← Counter (DBC)"
        elif 56 <= bit_pos <= 63:
            note = "  ← CheckSum (DBC)"
        print(f"  bit{bit_pos:2d} (b{byte_i}.bit{bit_i}): {cnt:5d} 次{note}")

    # 排除 Counter/Checksum 的活跃 bit（这些就算全动也没意义）
    non_cc = [(i, c) for i, c in active if not (52 <= i <= 63)]
    if non_cc:
        print(f"\n[关键发现] 除 Counter/Checksum 外,以下 bit 在录制中有翻转(这些就是可能的状态信号):")
        for bit_pos, cnt in non_cc:
            byte_i, bit_i = bit_pos // 8, bit_pos % 8
            print(f"  bit{bit_pos} (b{byte_i}.bit{bit_i}): {cnt} 次")
    else:
        print(f"\n[关键发现] 除 Counter/Checksum 外,所有 bit 都静态不动.")
        print(f"  → MPC 在这段录制中从未报告任何状态变化 → LKA 开关不在这段录制里切过")

    print(f"\n[差异帧序列 前20条]")
    for tag, idx, dat, flips in diff_log[:20]:
        if tag == 'INIT':
            print(f"  #{idx:5d} INIT {dat.hex(' ')}")
        else:
            flip_str = ','.join(f"b{bi}.bit{ii}" for bi, ii in flips)
            print(f"  #{idx:5d}      {dat.hex(' ')}  flip[{flip_str}]")

if __name__ == '__main__':
    main()
