#pragma once

#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/qt/network/wifi_manager.h"

#include "frogpilot/ui/qt/widgets/frogpilot_controls.h"

struct FrogPilotUIScene {
  bool enabled;
  bool frogpilot_panel_active;
  bool online;
  bool parked;
  bool reverse;
  bool standstill;

  int started_timer;
};

class FrogPilotUIState : public QObject {
  Q_OBJECT

public:
  explicit FrogPilotUIState(QObject *parent = nullptr);

  void update();

  std::unique_ptr<SubMaster> sm;

  FrogPilotUIScene frogpilot_scene;

  Params params;

  WifiManager *wifi;

signals:
};

FrogPilotUIState *frogpilotUIState();
