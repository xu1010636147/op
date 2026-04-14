#include "selfdrive/ui/qt/qt_window.h"

void setMainWindow(QWidget *w) {
  const float scale = util::getenv("SCALE", 1.0f);
  const QSize sz = QGuiApplication::primaryScreen()->size();

  if (Hardware::PC()) {
    w->resize(1920, 1080); 
    w->setWindowState(Qt::WindowFullScreen);
    w->show();
  } else {
    // 👇 车机设备：保持原逻辑（全屏 + 固定大小）
    if (scale == 1.0 && !(sz - DEVICE_SCREEN_SIZE).isValid()) {
      w->setFixedSize(DEVICE_SCREEN_SIZE);
      w->setWindowState(Qt::WindowFullScreen);
    } else {
      w->setFixedSize(DEVICE_SCREEN_SIZE * scale);
    }
    w->show();

#ifdef QCOM2
    QPlatformNativeInterface *native = QGuiApplication::platformNativeInterface();
    wl_surface *s = reinterpret_cast<wl_surface*>(native->nativeResourceForWindow("surface", w->windowHandle()));
    wl_surface_set_buffer_transform(s, WL_OUTPUT_TRANSFORM_270);
    wl_surface_commit(s);

    w->setWindowState(Qt::WindowFullScreen);
    w->setVisible(true);

    void *egl = native->nativeResourceForWindow("egldisplay", w->windowHandle());
    assert(egl != nullptr);
#endif
  }
}
