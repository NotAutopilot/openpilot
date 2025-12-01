#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotNavigationPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotNavigationPanel(FrogPilotSettingsWindow *parent, bool forceOpen = false);

signals:
  void closeSubPanel();
  void openSubPanel();

protected:
  void hideEvent(QHideEvent *event);
  void showEvent(QShowEvent *event) override;

private:
  void mousePressEvent(QMouseEvent *event);
  void updateButtons();
  void updateState(const UIState &s, const FrogPilotUIState &fs);
  void updateStep();

  bool forceOpenDescriptions;
  bool mapboxPublicKeySet;
  bool mapboxSecretKeySet;
  bool updatingLimits;

  FrogPilotButtonsControl *publicMapboxKeyControl;
  FrogPilotButtonsControl *secretMapboxKeyControl;
  ButtonControl *setupButton;

  FrogPilotButtonControl *updateSpeedLimitsToggle;

  FrogPilotSettingsWindow *parent;

  LabelControl *ipLabel;

  Params params;

  QLabel *imageLabel;

  QNetworkAccessManager *networkManager;

  QStackedLayout *primelessLayout;
};
