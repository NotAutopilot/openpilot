#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotModelPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotModelPanel(FrogPilotSettingsWindow *parent, bool forceOpen = false);

signals:
  void openSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateModelLabels(FrogPilotListWidget *labelsList);
  void updateState(const UIState &s, const FrogPilotUIState &fs);
  void updateToggles();

  bool allModelsDownloaded;
  bool allModelsDownloading;
  bool cancellingDownload;
  bool finalizingDownload;
  bool forceOpenDescriptions;
  bool modelDownloading;
  bool noModelsDownloaded;
  bool started;
  bool tinygradUpdate;
  bool updatingTinygrad;

  std::map<QString, AbstractControl*> toggles;

  ButtonControl *selectModelButton;

  FrogPilotButtonsControl *deleteModelButton;
  FrogPilotButtonsControl *downloadModelButton;
  FrogPilotButtonsControl *updateTinygradButton;

  FrogPilotSettingsWindow *parent;

  Params params;

  QDir modelDir{"/data/models/"};

  QMap<QString, QString> modelFileToNameMap;
  QMap<QString, QString> modelFileToNameMapProcessed;

  QString currentModel;
  QString defaultModel;

  QStringList availableModelNames;
};
