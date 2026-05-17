#ifdef QCOM2
// 高通平台摄像头公共代码
#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <string>

#include "common/swaglog.h"
#include "system/camerad/cameras/spectra.h"


void CameraBuf::init(cl_device_id device_id, cl_context context, SpectraCamera *cam, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  vipc_server = v;
  stream_type = type;
  frame_buf_count = frame_cnt;

  const SensorInfo *sensor = cam->sensor.get();

  // RAW frames from ISP
  if (cam->cc.output_type != ISP_IFE_PROCESSED) {
    camera_bufs_raw = std::make_unique<VisionBuf[]>(frame_buf_count);
  }
}

CameraBuf::~CameraBuf() {
  vipc_server = nullptr;
}
#else
// PC/通用平台
#include "system/camerad/cameras/camera_common.h"

CameraBuf::~CameraBuf() {
  vipc_server = nullptr;
}

void CameraBuf::init(cl_device_id device_id, cl_context context, SpectraCamera *cam, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  // Not used on PC
}
#endif
