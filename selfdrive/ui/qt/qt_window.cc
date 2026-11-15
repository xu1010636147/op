#include "selfdrive/ui/qt/qt_window.h"

#include <QTimer>

void setMainWindow(QWidget *w) {
  const float scale = util::getenv("SCALE", 1.0f);
  const QScreen *screen = QGuiApplication::primaryScreen();
  const QSize sz = screen->size();

  if (Hardware::PC() && scale == 1.0 && !(sz - DEVICE_SCREEN_SIZE).isValid()) {
    w->setMinimumSize(QSize(640, 480)); // allow resize smaller than fullscreen
    w->setMaximumSize(DEVICE_SCREEN_SIZE);
    w->resize(sz);
  } else {
    w->setFixedSize(DEVICE_SCREEN_SIZE * scale);
  }
  w->show();

  // Mutter/GNOME ignores a position set before show() and places the window itself once
  // it is mapped (typically on the monitor under the pointer), so pin it to the primary
  // screen after it has been shown.
  if (Hardware::PC()) {
    QTimer::singleShot(200, w, [w]() {
      if (const QScreen *s = QGuiApplication::primaryScreen()) {
        w->move(s->geometry().topLeft());
      }
    });
  }

#ifdef QCOM2
  QPlatformNativeInterface *native = QGuiApplication::platformNativeInterface();
  wl_surface *s = reinterpret_cast<wl_surface*>(native->nativeResourceForWindow("surface", w->windowHandle()));
  wl_surface_set_buffer_transform(s, WL_OUTPUT_TRANSFORM_270);
  wl_surface_commit(s);

  w->setWindowState(Qt::WindowFullScreen);
  w->setVisible(true);

  // ensure we have a valid eglDisplay, otherwise the ui will silently fail
  void *egl = native->nativeResourceForWindow("egldisplay", w->windowHandle());
  assert(egl != nullptr);
#endif
}


extern "C" {
  void set_main_window(void *w) {
    setMainWindow((QWidget*)w);
  }
}
