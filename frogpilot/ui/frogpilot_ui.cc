#include "frogpilot/ui/frogpilot_ui.h"

static void update_state(FrogPilotUIState *fs) {
  FrogPilotUIScene &frogpilot_scene = fs->frogpilot_scene;

  SubMaster &fpsm = *(fs->sm);
  fpsm.update(0);

  if (fpsm.updated("deviceState")) {
    const cereal::DeviceState::Reader &deviceState = fpsm["deviceState"].getDeviceState();
    frogpilot_scene.online = deviceState.getNetworkType() != cereal::DeviceState::NetworkType::NONE;
  }
  if (fpsm.updated("frogpilotCarState")) {
    const cereal::FrogPilotCarState::Reader &frogpilotCarState = fpsm["frogpilotCarState"].getFrogpilotCarState();
    frogpilot_scene.always_on_lateral_active = !frogpilot_scene.enabled && frogpilotCarState.getAlwaysOnLateralEnabled();
  }
  if (fpsm.updated("frogpilotPlan")) {
    const cereal::FrogPilotPlan::Reader &frogpilotPlan = fpsm["frogpilotPlan"].getFrogpilotPlan();
    if (frogpilotPlan.getThemeUpdated()) {
      emit fs->themeUpdated();
    }
    capnp::Text::Reader toggles = frogpilotPlan.getFrogpilotToggles();
    QByteArray current_toggles(toggles.cStr(), toggles.size());
    static QByteArray previous_toggles;
    if (previous_toggles != current_toggles) {
      frogpilot_scene.frogpilot_toggles = QJsonDocument::fromJson(current_toggles).object();
      previous_toggles = current_toggles;
    }
  }
  if (fpsm.updated("selfdriveState")) {
    const cereal::SelfdriveState::Reader &selfdriveState = fpsm["selfdriveState"].getSelfdriveState();
    frogpilot_scene.enabled = selfdriveState.getEnabled();
  }
}

FrogPilotUIState::FrogPilotUIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "carControl", "deviceState", "frogpilotCarState", "frogpilotDeviceState",
    "frogpilotPlan", "frogpilotRadarState", "frogpilotSelfdriveState", "liveDelay",
    "liveParameters", "liveTorqueParameters", "liveTracks", "selfdriveState"
  });

  wifi = new WifiManager(this);

  if (params.getInt("TetheringEnabled") == 1) {
    wifi->setTetheringEnabled(true);
  }
}

FrogPilotUIState *frogpilotUIState() {
  static FrogPilotUIState frogpilot_ui_state;
  return &frogpilot_ui_state;
}

void FrogPilotUIState::update() {
  update_state(this);

  frogpilot_scene.conditional_status = frogpilot_scene.enabled ? params_memory.getInt("CEStatus") : 0;
  frogpilot_scene.driver_camera_timer = frogpilot_scene.reverse && frogpilot_scene.frogpilot_toggles.value("driver_camera_in_reverse").toBool() ? frogpilot_scene.driver_camera_timer + 1 : 0;
}
