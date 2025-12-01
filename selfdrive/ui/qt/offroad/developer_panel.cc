#include "selfdrive/ui/qt/offroad/developer_panel.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/controls.h"

DeveloperPanel::DeveloperPanel(SettingsWindow *parent) : QFrame(parent) {
  mainLayout = new QStackedLayout(this);

  mainWidget = new QWidget(this);
  QVBoxLayout *mainListLayout = new QVBoxLayout(mainWidget);

  mainListLayout->setContentsMargins(50, 25, 50, 25);
  mainListLayout->setSpacing(20);

  FrogPilotListWidget *mainList = new FrogPilotListWidget(mainWidget);
  mainListLayout->addWidget(mainList);

  adbToggle = new ParamControl("AdbEnabled", tr("Enable ADB"),
            tr("ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. See https://docs.comma.ai/how-to/connect-to-comma for more info."), "");
  mainList->addItem(adbToggle);

  // SSH keys
  mainList->addItem(new SshToggle());
  mainList->addItem(new SshControl());

  joystickToggle = new ParamControl("JoystickDebugMode", tr("Joystick Debug Mode"), "", "");
  QObject::connect(joystickToggle, &ParamControl::toggleFlipped, [=](bool state) {
    params.putBool("LongitudinalManeuverMode", false);
    longManeuverToggle->refresh();
  });
  mainList->addItem(joystickToggle);

  longManeuverToggle = new ParamControl("LongitudinalManeuverMode", tr("Longitudinal Maneuver Mode"), "", "");
  QObject::connect(longManeuverToggle, &ParamControl::toggleFlipped, [=](bool state) {
    params.putBool("JoystickDebugMode", false);
    joystickToggle->refresh();
  });
  mainList->addItem(longManeuverToggle);

  experimentalLongitudinalToggle = new ParamControl(
    "AlphaLongitudinalEnabled",
    tr("openpilot Longitudinal Control (Alpha)"),
    QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
              "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
    ""
  );
  experimentalLongitudinalToggle->setConfirmation(true, false);
  QObject::connect(experimentalLongitudinalToggle, &ParamControl::toggleFlipped, [=]() {
    updateToggles(offroad);
  });
  mainList->addItem(experimentalLongitudinalToggle);

  // Joystick and longitudinal maneuvers should be hidden on release branches
  is_release = params.getBool("IsReleaseBranch");

  // Toggles should be not available to change in onroad state
  QObject::connect(uiState(), &UIState::offroadTransition, this, &DeveloperPanel::updateToggles);

  // FrogPilot variables
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
  }

  std::vector<std::string> keys = params.allKeys();
  for (const std::string &key : keys) {
    frogpilotToggleLevels[QString::fromStdString(key)] = params.getTuningLevel(key);
  }

  developerUIToggle = new FrogPilotManageControl("DeveloperUI", tr("Developer UI"), tr("<b>Detailed information about openpilot's internal operations.</b>"), "");
  QObject::connect(developerUIToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
    mainLayout->setCurrentWidget(developerUIPanel);
    emit openSubPanel();
  });
  mainList->insertItem(0, developerUIToggle);

  mainLayout->addWidget(mainWidget);

  FrogPilotListWidget *developerUIList = new FrogPilotListWidget(this);
  developerUIList->setContentsMargins(50, 25, 50, 25);
  developerUIPanel = new ScrollView(developerUIList, this);
  mainLayout->addWidget(developerUIPanel);

  FrogPilotListWidget *developerMetricList = new FrogPilotListWidget(this);
  developerMetricList->setContentsMargins(50, 25, 50, 25);
  developerMetricPanel = new ScrollView(developerMetricList, this);
  mainLayout->addWidget(developerMetricPanel);

  FrogPilotListWidget *developerSidebarList = new FrogPilotListWidget(this);
  developerSidebarList->setContentsMargins(50, 25, 50, 25);
  developerSidebarPanel = new ScrollView(developerSidebarList, this);
  mainLayout->addWidget(developerSidebarPanel);

  FrogPilotListWidget *developerWidgetList = new FrogPilotListWidget(this);
  developerWidgetList->setContentsMargins(50, 25, 50, 25);
  developerWidgetPanel = new ScrollView(developerWidgetList, this);
  mainLayout->addWidget(developerWidgetPanel);

  std::vector<std::tuple<QString, QString, QString, QString>> developerToggles {
    {"DeveloperMetrics", tr("Developer Metrics"), tr("<b>Performance data, sensor readings, and system metrics</b> for debugging and optimizing openpilot."), ""},
    {"AdjacentPathMetrics", tr("Adjacent Lane Metrics"), tr("<b>Show the width of the adjacent lanes.</b>"), ""},
    {"BorderMetrics", tr("Border Metrics"), tr("<b>Show statuses along the border of the driving screen.</b><br><br><b>Blind Spot</b>: The border turns red when a vehicle is in a blind spot<br><b>Steering Torque</b>: The border goes from green to red according to how much steering torque is being used<br><b>Turn Signal</b>: The border flashes yellow when a turn signal is on"), ""},
    {"FPSCounter", tr("FPS Display"), tr("<b>Show the frames per second (FPS)</b> at the bottom of the driving screen."), ""},
    {"LeadInfo", tr("Lead Info"), tr("<b>Show each tracked vehicle's distance and speed</b> below its marker."), ""},
    {"NumericalTemp", tr("Numerical Temperature Gauge"), tr("<b>Show a numerical temperature in the sidebar</b> instead of the status labels."), ""},
    {"SidebarMetrics", tr("Sidebar Metrics"), tr("<b>Display system information</b> (CPU, GPU, RAM usage, IP address, device storage) in the sidebar."), ""},
    {"UseSI", tr("Use International System of Units"), tr("<b>Display measurements using the \"International System of Units\" (SI)</b> standard."), ""},

    {"DeveloperSidebar", tr("Developer Sidebar"), tr("<b>Display debugging info and metrics</b> in a dedicated sidebar on the right side of the screen."), ""},
    {"DeveloperSidebarMetric1", tr("Metric #1"), tr("<b>Select the metric shown in the first \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric2", tr("Metric #2"), tr("<b>Select the metric shown in the second \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric3", tr("Metric #3"), tr("<b>Select the metric shown in the third \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric4", tr("Metric #4"), tr("<b>Select the metric shown in the fourth \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric5", tr("Metric #5"), tr("<b>Select the metric shown in the fifth \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric6", tr("Metric #6"), tr("<b>Select the metric shown in the sixth \"Developer Sidebar\" widget.</b>"), ""},
    {"DeveloperSidebarMetric7", tr("Metric #7"), tr("<b>Select the metric shown in the seventh \"Developer Sidebar\" widget.</b>"), ""},

    {"DeveloperWidgets", tr("Developer Widgets"), tr("<b>Overlays for debugging visuals, internal states, and model predictions</b> on the driving screen."), ""},
    {"AdjacentLeadsUI", tr("Adjacent Leads Tracking"), tr("<b>Display adjacent leads detected by the car's radar</b> to the left and right of the current driving path."), ""},
    {"ShowStoppingPoint", tr("Model Stopping Point"), tr("<b>Show a stop-sign marker where the model intends to stop.</b>"), ""},
    {"RadarTracksUI", tr("Radar Tracks"), tr("<b>Display all radar points</b> produced by the car's radar."), ""},
  };

  for (const auto &[param, title, desc, icon] : developerToggles) {
    AbstractControl *developerToggle;

    if (param == "DeveloperMetrics") {
      FrogPilotManageControl *developerMetricsToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(developerMetricsToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
        mainLayout->setCurrentWidget(developerMetricPanel);
        emit openSubSubPanel();
      });
      developerToggle = developerMetricsToggle;
    } else if (param == "BorderMetrics") {
      std::vector<QString> borderToggles{"BlindSpotMetrics", "ShowSteering", "SignalMetrics"};
      std::vector<QString> borderToggleNames{tr("Blind Spot"), tr("Steering Torque"), tr("Turn Signal")};
      borderMetricsButton = new FrogPilotButtonToggleControl(param, title, desc, icon, borderToggles, borderToggleNames);
      developerToggle = borderMetricsButton;
    } else if (param == "NumericalTemp") {
      std::vector<QString> temperatureToggles{"Fahrenheit"};
      std::vector<QString> temperatureToggleNames{tr("Fahrenheit")};
      developerToggle = new FrogPilotButtonToggleControl(param, title, desc, icon, temperatureToggles, temperatureToggleNames);
    } else if (param == "SidebarMetrics") {
      sidebarMetricsToggles = {"ShowCPU", "ShowGPU", "ShowIP", "ShowMemoryUsage", "ShowStorageLeft", "ShowStorageUsed"};
      std::vector<QString> sidebarMetricsToggleNames{tr("CPU"), tr("GPU"), tr("IP"), tr("RAM"), tr("SSD Left"), tr("SSD Used")};
      sidebarMetricsToggle = new FrogPilotButtonsControl(title, desc, icon, sidebarMetricsToggleNames, true, false, 150);
      for (int i = 0; i < sidebarMetricsToggles.size(); ++i) {
        if (params.getBool(sidebarMetricsToggles[i].toStdString())) {
          sidebarMetricsToggle->setCheckedButton(i);
        }
      }
      QObject::connect(sidebarMetricsToggle, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
        params.putBool(sidebarMetricsToggles[id].toStdString(), !params.getBool(sidebarMetricsToggles[id].toStdString()));

        if (id == 0) {
          params.putBool("ShowGPU", false);
        } else if (id == 1) {
          params.putBool("ShowCPU", false);
        } else if (id == 3) {
          params.putBool("ShowStorageLeft", false);
          params.putBool("ShowStorageUsed", false);
        } else if (id == 4) {
          params.putBool("ShowMemoryUsage", false);
          params.putBool("ShowStorageUsed", false);
        } else if (id == 5) {
          params.putBool("ShowMemoryUsage", false);
          params.putBool("ShowStorageLeft", false);
        }

        sidebarMetricsToggle->clearCheckedButtons();
        for (int i = 0; i < sidebarMetricsToggles.size(); ++i) {
          if (params.getBool(sidebarMetricsToggles[i].toStdString())) {
            sidebarMetricsToggle->setCheckedButton(i);
          }
        }
      });
      developerToggle = sidebarMetricsToggle;
    } else if (param == "DeveloperSidebar") {
      FrogPilotManageControl *developerSidebarToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(developerSidebarToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
        mainLayout->setCurrentWidget(developerSidebarPanel);
        emit openSubSubPanel();
      });
      developerToggle = developerSidebarToggle;
    } else if (developerSidebarKeys.contains(param)) {
      QMap<int, QString> developerSidebarMetricOptions {
        {0, tr("None")},
        {1, tr("Acceleration: Current")},
        {2, tr("Acceleration: Max")},
        {3, tr("Auto Tune: Actuator Delay")},
        {4, tr("Auto Tune: Friction")},
        {5, tr("Auto Tune: Lateral Acceleration")},
        {6, tr("Auto Tune: Steer Ratio")},
        {7, tr("Auto Tune: Stiffness Factor")},
        {8, tr("Engagement %: Lateral")},
        {9, tr("Engagement %: Longitudinal")},
        {10, tr("Lateral Control: Steering Angle")},
        {11, tr("Lateral Control: Torque % Used")},
        {12, tr("Longitudinal Control: Actuator Acceleration Output")},
        {13, tr("Longitudinal MPC: Danger Factor")},
        {14, tr("Longitudinal MPC Jerk: Acceleration")},
        {15, tr("Longitudinal MPC Jerk: Danger Zone")},
        {16, tr("Longitudinal MPC Jerk: Speed Control")},
      };

      ButtonControl *metricToggle = new ButtonControl(title, tr("SELECT"), desc);
      QObject::connect(metricToggle, &ButtonControl::clicked, [metricToggle, key = param, developerSidebarMetricOptions, this]() mutable {
        QString current = developerSidebarMetricOptions.value(params.getInt(key.toStdString()), tr("None"));
        QString selection = MultiOptionDialog::getSelection(tr("Select a metric to display"), developerSidebarMetricOptions.values(), current, this);

        if (!selection.isEmpty()) {
          int selectedMetric = developerSidebarMetricOptions.key(selection);

          params.putInt(key.toStdString(), selectedMetric);

          metricToggle->setValue(selection);
        }
      });
      metricToggle->setValue(developerSidebarMetricOptions.value(params.getInt(param.toStdString()), tr("None")));
      developerToggle = metricToggle;
    } else if (param == "DeveloperWidgets") {
      FrogPilotManageControl *developerWidgetsToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(developerWidgetsToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
        mainLayout->setCurrentWidget(developerWidgetPanel);
        emit openSubSubPanel();
      });
      developerToggle = developerWidgetsToggle;
    } else if (param == "ShowStoppingPoint") {
      std::vector<QString> stoppingPointToggles{"ShowStoppingPointMetrics"};
      std::vector<QString> stoppingPointToggleNames{tr("Show Distance")};
      developerToggle = new FrogPilotButtonToggleControl(param, title, desc, icon, stoppingPointToggles, stoppingPointToggleNames);
    } else {
      developerToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = developerToggle;

    if (developerMetricKeys.contains(param)) {
      developerMetricList->addItem(developerToggle);
    } else if (developerSidebarKeys.contains(param)) {
      developerSidebarList->addItem(developerToggle);
    } else if (developerWidgetKeys.contains(param)) {
      developerWidgetList->addItem(developerToggle);
    } else {
      developerUIList->addItem(developerToggle);

      parentKeys.insert(param);
    }
  }

  QObject::connect(parent, &SettingsWindow::closeSubPanel, [this]() {mainLayout->setCurrentWidget(mainWidget);});
  QObject::connect(parent, &SettingsWindow::closeSubSubPanel, [this]() {mainLayout->setCurrentWidget(developerUIPanel);});
}

void DeveloperPanel::updateToggles(bool _offroad) {
  for (auto btn : findChildren<ParamControl *>()) {
    btn->setVisible(!is_release);

    /*
     * experimentalLongitudinalToggle should be toggelable when:
     * - visible, and
     * - during onroad & offroad states
     */
    if (btn != experimentalLongitudinalToggle) {
      btn->setEnabled(_offroad);
    }

    // FrogPilot variables
    for (auto &[key, toggle] : toggles) {
      if (toggle == btn) {
        btn->setEnabled(true);
        break;
      }
    }
    if (developerUIToggle == btn) {
      btn->setEnabled(true);
    }
  }

  // longManeuverToggle and experimentalLongitudinalToggle should not be toggleable if the car does not have longitudinal control
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getAlphaLongitudinalAvailable() || is_release) {
      params.remove("AlphaLongitudinalEnabled");
      experimentalLongitudinalToggle->setEnabled(false);
    }

    /*
     * experimentalLongitudinalToggle should be visible when:
     * - is not a release branch, and
     * - the car supports experimental longitudinal control (alpha)
     */
    experimentalLongitudinalToggle->setVisible(CP.getAlphaLongitudinalAvailable() && !is_release);

    longManeuverToggle->setEnabled(hasLongitudinalControl(CP) && _offroad);

    // FrogPilot variables
    hasOpenpilotLongitudinal = hasLongitudinalControl(CP);
    hasRadar = !CP.getRadarUnavailable();

    borderMetricsButton->setVisibleButton(0, CP.getEnableBsm());
  } else {
    longManeuverToggle->setEnabled(false);
    experimentalLongitudinalToggle->setVisible(false);
  }
  experimentalLongitudinalToggle->refresh();

  offroad = _offroad;

  // FrogPilot variables
  tuningLevel = params.getInt("TuningLevel");

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      toggle->setVisible(false);
    }
  }

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      continue;
    }

    bool setVisible = tuningLevel >= frogpilotToggleLevels[key].toDouble();

    if (key == "AdjacentLeadsUI") {
      setVisible &= hasRadar && !(params.getBool("AdvancedCustomUI") && params.getBool("HideLeadMarker"));
    }

    else if (key == "RadarTracksUI") {
      setVisible &= hasRadar;
    }

    else if (key == "ShowStoppingPoint") {
      setVisible &= hasOpenpilotLongitudinal;
    }

    toggle->setVisible(setVisible);

    if (setVisible) {
      if (developerMetricKeys.contains(key)) {
        toggles["DeveloperMetrics"]->setVisible(true);
      } else if (developerSidebarKeys.contains(key)) {
        toggles["DeveloperSidebar"]->setVisible(true);
      } else if (developerUIKeys.contains(key)) {
        toggles["DeveloperUI"]->setVisible(true);
      } else if (developerWidgetKeys.contains(key)) {
        toggles["DeveloperWidgets"]->setVisible(true);
      }
    }
  }

  borderMetricsButton->setVisibleButton(0, hasBSM);

  developerUIToggle->setVisible(tuningLevel >= frogpilotToggleLevels["DeveloperUI"].toDouble());

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}

void DeveloperPanel::showEvent(QShowEvent *event) {
  updateToggles(offroad);

  // FrogPilot variables
  for (int i = 0; i < sidebarMetricsToggles.size(); ++i) {
    if (params.getBool(sidebarMetricsToggles[i].toStdString())) {
      sidebarMetricsToggle->setCheckedButton(i);
    }
  }

  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }
}
