#pragma once

#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

#include "frogpilot/ui/frogpilot_ui.h"
#include "frogpilot/ui/qt/widgets/frogpilot_controls.h"

class QNetworkAccessManager;

class FrogPilotSettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit FrogPilotSettingsWindow(SettingsWindow *parent);

  void updateVariables();

  bool canUsePedal = false;
  bool canUseSDSU = false;
  bool forceOpenDescriptions = false;
  bool hasAlphaLongitudinal = false;
  bool hasAutoTune = true;
  bool hasBSM = true;
  bool hasDashSpeedLimits = true;
  bool hasNNFFLog = true;
  bool hasOpenpilotLongitudinal = true;
  bool hasPCMCruise = false;
  bool hasPedal = false;
  bool hasRadar = true;
  bool hasSDSU = false;
  bool hasSNG = false;
  bool hasZSS = false;
  bool isAngleCar = false;
  bool isBolt = false;
  bool isFrogsGoMoo = false;
  bool isGM = true;
  bool isHKG = true;
  bool isHKGCanFd = true;
  bool isSubaru = false;
  bool isTorqueCar = false;
  bool isToyota = true;
  bool isTSK = false;
  bool isVolt = true;
  bool keepScreenOn = false;
  bool lkasAllowedForAOL = false;
  bool openpilotLongitudinalControlDisabled = false;

  float friction;
  float latAccelFactor;
  float longitudinalActuatorDelay;
  float startAccel;
  float steerActuatorDelay;
  float steerKp;
  float steerRatio;
  float stopAccel;
  float stoppingDecelRate;
  float vEgoStarting;
  float vEgoStopping;

  int tuningLevel;

  QJsonObject frogpilotToggleLevels;

signals:
  void closeSubPanel();
  void closeSubSubPanel();
  void closeSubSubSubPanel();
  void openPanel();
  void openSubPanel();
  void openSubSubPanel();
  void openSubSubSubPanel();
  void tuningLevelChanged(int level);
  void updateMetric(bool metric, bool bootRun=false);

private:
  void closePanel();
  void createPanelButtons(FrogPilotListWidget *list);
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void updateState();
  void updateTuningLevel();

  bool panelOpen;

  std::string carMake;

  FrogPilotButtonsControl *drivingPanelButtons;
  FrogPilotButtonsControl *navigationPanelButtons;
  FrogPilotButtonsControl *soundPanelButtons;
  FrogPilotButtonsControl *systemPanelButtons;
  FrogPilotButtonsControl *themePanelButtons;
  FrogPilotButtonsControl *togglePreset;
  FrogPilotButtonsControl *vehiclePanelButtons;

  Params params;

  QJsonObject shownDescriptions;

  QStackedLayout *mainLayout;

  ScrollView *frogpilotPanel;
};
