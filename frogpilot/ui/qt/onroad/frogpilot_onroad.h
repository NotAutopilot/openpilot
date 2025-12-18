#pragma once

#include "selfdrive/ui/qt/onroad/annotated_camera.h"

class FrogPilotOnroadWindow : public QWidget {
  Q_OBJECT

public:
  FrogPilotOnroadWindow(QWidget* parent = 0);

  void updateState(const UIState &s, const FrogPilotUIState &fs);

  float fps;

  FrogPilotUIScene frogpilot_scene;

  QColor bg;

  QJsonObject frogpilot_toggles;

private:
  void paintEvent(QPaintEvent *event);
  void paintFPS(QPainter &p);
  void paintSteeringTorqueBorder(QPainter &p);
  void paintTurnSignalBorder(QPainter &p);
  void resizeEvent(QResizeEvent *event);

  bool blindSpotLeft;
  bool blindSpotRight;
  bool flickerActive;
  bool showBlindspot;
  bool showFPS;
  bool showSignal;
  bool showSteering;
  bool turnSignalLeft;
  bool turnSignalRight;

  float smoothedSteer;
  float torque;

  QColor leftBorderColor;
  QColor rightBorderColor;

  QRect rect;

  QRegion marginRegion;

  QString fpsDisplayString;

  QTimer *signalTimer;
};
