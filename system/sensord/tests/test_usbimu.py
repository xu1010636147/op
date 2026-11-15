import struct

import pytest

from openpilot.system.sensord.sensord_usbimu import (
  CHECKSUM_OFFSET, FRAME_SIZE, extract_frames, decode_frame, map_axes,
  ACCEL_SCALE, GYRO_SCALE, SENSOR_ACCELEROMETER, SENSOR_TYPE_ACCELEROMETER,
  SENSOR_GYRO_UNCALIBRATED, SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
)


def make_frame(seq, ax, ay, az, gx, gy, gz):
  """按协议手工构造一帧，用于验证接收侧的解析。"""
  frame = bytearray(FRAME_SIZE)
  frame[0] = 0xAA
  frame[1] = 0x55
  frame[2] = seq
  struct.pack_into('<hhh', frame, 3, ax, ay, az)
  struct.pack_into('<hhh', frame, 9, gx, gy, gz)
  frame[CHECKSUM_OFFSET] = sum(frame[:CHECKSUM_OFFSET]) & 0xFF
  return bytes(frame)


class TestExtractFrames:
  def test_single_frame(self):
    buf = bytearray(make_frame(1, 100, 200, 300, 10, 20, 30))
    frames = extract_frames(buf)
    assert len(frames) == 1
    assert buf == bytearray()

  def test_multiple_frames_in_one_chunk(self):
    buf = bytearray()
    for seq in range(5):
      buf.extend(make_frame(seq, seq, 0, 0, 0, 0, 0))
    frames = extract_frames(buf)
    assert len(frames) == 5
    assert [f[2] for f in frames] == [0, 1, 2, 3, 4]
    assert buf == bytearray()

  def test_truncated_frame_is_kept(self):
    """数据还没收齐时，半帧必须留在缓冲区等下一批。"""
    whole = make_frame(7, 1, 2, 3, 4, 5, 6)
    buf = bytearray(whole[:10])
    assert extract_frames(buf) == []
    assert bytes(buf) == whole[:10]

    # 补齐剩余字节后应当解析出来
    buf.extend(whole[10:])
    frames = extract_frames(buf)
    assert len(frames) == 1

  def test_split_header_across_reads(self):
    """帧头被拆到两次 read 之间是常态，不能丢掉。"""
    buf = bytearray(b'\xaa')
    assert extract_frames(buf) == []
    assert bytes(buf) == b'\xaa'

    buf.extend(b'\x55' + make_frame(9, 0, 0, 0, 0, 0, 0)[2:])
    assert len(extract_frames(buf)) == 1

  def test_leading_noise_is_discarded(self):
    buf = bytearray(b'\xde\xad\xbe\xef' + make_frame(3, 1, 1, 1, 1, 1, 1))
    frames = extract_frames(buf)
    assert len(frames) == 1
    assert decode_frame(frames[0])[0] == 3

  def test_bad_checksum_resyncs_without_losing_next_frame(self):
    """校验失败的帧只丢 1 字节，后续合法帧必须还能被找到。"""
    bad = bytearray(make_frame(1, 0, 0, 0, 0, 0, 0))
    bad[CHECKSUM_OFFSET] ^= 0xFF   # 破坏校验和

    good = make_frame(2, 5, 6, 7, 8, 9, 10)
    buf = bytearray(bytes(bad) + good)

    frames = extract_frames(buf)
    assert len(frames) == 1
    assert decode_frame(frames[0])[0] == 2
    assert buf == bytearray()

  def test_noise_containing_partial_header_is_trimmed(self):
    """无帧头时缓冲区不能无限增长，只留最后 1 字节。"""
    buf = bytearray(b'\x01\x02\x03\xaa')
    assert extract_frames(buf) == []
    assert bytes(buf) == b'\xaa'


class TestDecodeFrame:
  def test_roundtrip(self):
    frame = make_frame(42, -32768, 32767, -1, 1234, -5678, 0)
    seq, ax, ay, az, gx, gy, gz = decode_frame(frame)
    assert seq == 42
    assert (ax, ay, az) == (-32768, 32767, -1)
    assert (gx, gy, gz) == (1234, -5678, 0)

  def test_scale_factors_match_full_scale(self):
    """±32768 原始值应对应满量程：±4g / ±500dps。"""
    assert ACCEL_SCALE * 32768 == pytest.approx(4.0 * 9.80665)
    assert GYRO_SCALE * 32768 == pytest.approx(500.0 * 3.141592653589793 / 180.0)

  def test_sensor_constants_match_sensord(self):
    """这些常量必须与 system/sensord/sensors/constants.h 一致，否则 locationd 会忽略消息。"""
    assert (SENSOR_ACCELEROMETER, SENSOR_TYPE_ACCELEROMETER) == (1, 1)
    assert (SENSOR_GYRO_UNCALIBRATED, SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) == (5, 16)


class TestMapAxes:
  def test_matches_onboard_lsm6ds3_mapping(self):
    """与 system/sensord/sensors/lsm6ds3_accel.cc 的 {y, -x, z} 保持一致。"""
    assert map_axes(1, 2, 3) == (2, -1, 3)

  def test_is_permutation_of_magnitude(self):
    """轴映射只做重排与取反，不能改变矢量的模长。"""
    x, y, z = 1.5, -2.5, 3.5
    mapped = map_axes(x, y, z)
    assert sum(v * v for v in mapped) == pytest.approx(x * x + y * y + z * z)
