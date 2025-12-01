#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotThemesPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotThemesPanel(FrogPilotSettingsWindow *parent, bool forceOpen = false);

protected:
  void showEvent(QShowEvent *event) override;

signals:
  void openSubPanel();

private:
  void updateState(const UIState &s, const FrogPilotUIState &fs);
  void updateToggles();

  bool cancellingDownload;
  bool colorDownloading;
  bool colorsDownloaded;
  bool distanceIconDownloading;
  bool distanceIconsDownloaded;
  bool finalizingDownload;
  bool forceOpenDescriptions;
  bool iconDownloading;
  bool iconsDownloaded;
  bool randomThemes;
  bool signalDownloading;
  bool signalsDownloaded;
  bool soundDownloading;
  bool soundsDownloaded;
  bool themeDownloading;
  bool wheelDownloading;
  bool wheelsDownloaded;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> customThemeKeys = {"ColorScheme", "DistanceIconPack", "DownloadStatusLabel", "IconPack", "SignalAnimation", "SoundPack", "WheelIcon"};

  QSet<QString> parentKeys;

  FrogPilotButtonsControl *manageColorSchemeButton;
  FrogPilotButtonsControl *manageDistanceIconPackButton;
  FrogPilotButtonsControl *manageIconPackButton;
  FrogPilotButtonsControl *manageSignalAnimationButton;
  FrogPilotButtonsControl *manageSoundPackButton;
  FrogPilotButtonsControl *manageWheelIconsButton;

  FrogPilotSettingsWindow *parent;

  LabelControl *downloadStatusLabel;

  QDir themePacksDirectory{"/data/themes/theme_packs/"};
  QDir wheelsDirectory{"/data/themes/steering_wheels/"};

  QString colorSchemeToDownload;
  QString distanceIconPackToDownload;
  QString iconPackToDownload;
  QString signalAnimationToDownload;
  QString soundPackToDownload;
  QString wheelToDownload;

  Params params;
};
