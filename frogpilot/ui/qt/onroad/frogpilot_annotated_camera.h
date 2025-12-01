#pragma once

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

const int widget_size = img_size + (UI_BORDER_SIZE / 2);

class FrogPilotAnnotatedCameraWidget : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotAnnotatedCameraWidget(QWidget *parent = 0);

  void mousePressEvent(QMouseEvent *mouseEvent) override;
  void paintBlindSpotPath(QPainter &p);
  void paintFrogPilotWidgets(QPainter &p, UIState &s);
  void updateState(const UIState &s, const FrogPilotUIState &fs);

  bool hideBottomIcons;
  bool isCruiseSet;
  bool rightHandDM;

  int alertHeight;
  int standstillDuration;

  float speed;

  FrogPilotUIScene frogpilot_scene;

  QColor blueColor(int alpha = 255) { return QColor(0, 0, 255, alpha); }
  QColor purpleColor(int alpha = 255) { return QColor(128, 0, 128, alpha); }
  QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }

  QJsonObject frogpilot_toggles;

  QPoint dmIconPosition;
  QPoint experimentalButtonPosition;

  QRect setSpeedRect;

  QSize defaultSize;

  QString signalStyle;

protected:
  void showEvent(QShowEvent *event) override;

private:
  void paintCEMStatus(QPainter &p);
  void paintCompass(QPainter &p);
  void paintCurveSpeedControl(QPainter &p);
  void paintCurveSpeedControlTraining(QPainter &p);
  void paintRoadName(QPainter &p);
  void paintStandstillTimer(QPainter &p);
  void paintStoppingPoint(QPainter &p);
  void paintTurnSignals(QPainter &p);
  void updateSignals();

  bool blindspotLeft;
  bool blindspotRight;
  bool blinkerLeft;
  bool blinkerRight;
  bool cscControllingSpeed;
  bool cscTraining;
  bool experimentalMode;
  bool redLight;

  int animationFrameIndex;
  int frogHopCount;
  int signalAnimationLength;
  int signalHeight;
  int signalMovement;
  int signalWidth;
  int totalFrames;

  float cscSpeed;
  float distanceConversion;
  float roadCurvature;
  float setSpeed;
  float speedConversion;
  float speedConversionMetrics;
  float stoppingDistance;

  Params params;
  Params params_memory{"", true};

  QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }

  QElapsedTimer glowTimer;
  QElapsedTimer standstillTimer;

  QPixmap curveSpeedIcon;
  QPixmap curveSpeedIconFlipped;
  QPixmap stopSignImg;

  QPoint cemStatusPosition;
  QPoint compassPosition;

  QSharedPointer<QMovie> cemCurveIcon;
  QSharedPointer<QMovie> cemLeadIcon;
  QSharedPointer<QMovie> cemSpeedIcon;
  QSharedPointer<QMovie> cemStopIcon;
  QSharedPointer<QMovie> cemTurnIcon;
  QSharedPointer<QMovie> chillModeIcon;
  QSharedPointer<QMovie> experimentalModeIcon;

  QString leadDistanceUnit;
  QString leadSpeedUnit;
  QString roadName;
  QString speedUnit;

  QTimer *animationTimer;

  QVector<QPixmap> blindspotImages;
  QVector<QPixmap> blindspotImagesRight;
  QVector<QPixmap> signalImages;
  QVector<QPixmap> signalImagesRight;
};
