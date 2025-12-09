#pragma once

#include "selfdrive/ui/qt/onroad/annotated_camera.h"

class FrogPilotOnroadWindow : public QWidget {
  Q_OBJECT

public:
  FrogPilotOnroadWindow(QWidget* parent = 0);

  void updateState(const UIState &s, const FrogPilotUIState &fs);

  FrogPilotUIScene frogpilot_scene;

  QColor bg;

  QJsonObject frogpilot_toggles;

private:
  void paintEvent(QPaintEvent *event);
  void resizeEvent(QResizeEvent *event);

  QRect rect;

  QRegion marginRegion;
};
