#include <QtConcurrent>

#include "frogpilot/ui/qt/offroad/maps_settings.h"

FrogPilotMapsPanel::FrogPilotMapsPanel(FrogPilotSettingsWindow *parent, bool forceOpen) : FrogPilotListWidget(parent), parent(parent) {
  forceOpenDescriptions = forceOpen;

  QStackedLayout *mapsLayout = new QStackedLayout();
  addItem(mapsLayout);

  FrogPilotListWidget *settingsList = new FrogPilotListWidget(this);

  std::vector<QString> scheduleOptions{tr("Manually"), tr("Weekly"), tr("Monthly")};
  preferredSchedule = new ButtonParamControl("PreferredSchedule", tr("Automatically Update Maps"),
                                          tr("<b>How often maps update</b> from \"OpenStreetMap (OSM)\" with the latest speed limit information. "
                                             "Weekly updates run every Sunday; monthly updates run on the 1st."),
                                             "",
                                             scheduleOptions);
  settingsList->addItem(preferredSchedule);

  downloadMapsButton = new ButtonControl(tr("Download Maps"), tr("DOWNLOAD"), tr("<b>Manually update your selected map sources</b> so \"Speed Limit Controller\" has the latest speed limit information."));
  QObject::connect(downloadMapsButton, &ButtonControl::clicked, [this] {
    if (downloadMapsButton->text() == tr("CANCEL")) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Cancel the download?"), this)) {
        cancelDownload();
      }
    } else {
      startDownload();
    }
  });
  settingsList->addItem(downloadMapsButton);

  settingsList->addItem(lastMapsDownload = new LabelControl(tr("Last Updated"), params.get("LastMapsUpdate").empty() ? "Never" : QString::fromStdString(params.get("LastMapsUpdate"))));

  selectMaps = new FrogPilotButtonsControl(tr("Map Sources"),
                                           tr("<b>Select the countries or U.S. states to use with \"Speed Limit Controller\".</b>") ,
                                              "", {tr("COUNTRIES"), tr("STATES")});
  QObject::connect(selectMaps, &FrogPilotButtonsControl::buttonClicked, [mapsLayout, this](int id) {
    mapsLayout->setCurrentIndex(id + 1);

    openSubPanel();
  });
  settingsList->addItem(selectMaps);

  settingsList->addItem(downloadStatus = new LabelControl(tr("Progress")));
  settingsList->addItem(downloadTimeElapsed = new LabelControl(tr("Time Elapsed")));
  settingsList->addItem(downloadETA = new LabelControl(tr("Time Remaining")));

  downloadETA->setVisible(false);
  downloadStatus->setVisible(false);
  downloadTimeElapsed->setVisible(false);

  removeMapsButton = new ButtonControl(tr("Remove Maps"), tr("REMOVE"), tr("<b>Delete downloaded map data</b> to free up storage space."));
  QObject::connect(removeMapsButton, &ButtonControl::clicked, [this] {
    if (FrogPilotConfirmationDialog::yesorno(tr("Delete all downloaded maps?"), this)) {
      std::thread([this] {
        mapsSize->setText(tr("0 MB"));

        mapsFolderPath.removeRecursively();
      }).detach();
    }
  });
  settingsList->addItem(removeMapsButton);

  settingsList->addItem(mapsSize = new LabelControl(tr("Storage Used"), calculateDirectorySize(mapsFolderPath)));

  ScrollView *settingsPanel = new ScrollView(settingsList, this);
  mapsLayout->addWidget(settingsPanel);

  FrogPilotListWidget *countriesList = new FrogPilotListWidget(this);
  std::vector<std::pair<QString, QMap<QString, QString>>> countries = {
    {tr("Africa"), africaMap},
    {tr("Antarctica"), antarcticaMap},
    {tr("Asia"), asiaMap},
    {tr("Europe"), europeMap},
    {tr("North America"), northAmericaMap},
    {tr("Oceania"), oceaniaMap},
    {tr("South America"), southAmericaMap}
  };

  for (std::pair<QString, QMap<QString, QString>> country : countries) {
    countriesList->addItem(new LabelControl(country.first, ""));
    countriesList->addItem(new MapSelectionControl(country.second, true));
  }

  ScrollView *countryMapsPanel = new ScrollView(countriesList, this);
  mapsLayout->addWidget(countryMapsPanel);

  FrogPilotListWidget *statesList = new FrogPilotListWidget(this);
  std::vector<std::pair<QString, QMap<QString, QString>>> states = {
    {tr("United States - Midwest"), midwestMap},
    {tr("United States - Northeast"), northeastMap},
    {tr("United States - South"), southMap},
    {tr("United States - West"), westMap},
    {tr("United States - Territories"), territoriesMap}
  };

  for (std::pair<QString, QMap<QString, QString>> state : states) {
    statesList->addItem(new LabelControl(state.first, ""));
    statesList->addItem(new MapSelectionControl(state.second));
  }

  ScrollView *stateMapsPanel = new ScrollView(statesList, this);
  mapsLayout->addWidget(stateMapsPanel);

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [mapsLayout, settingsPanel, this] {
    if (forceOpenDescriptions) {
      downloadMapsButton->showDescription();
      preferredSchedule->showDescription();
      removeMapsButton->showDescription();
      selectMaps->showDescription();
    }

    hasMapsSelected = !params.get("MapsSelected").empty();

    mapsLayout->setCurrentWidget(settingsPanel);
  });
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotMapsPanel::updateState);
}

void FrogPilotMapsPanel::showEvent(QShowEvent *event) {
  if (forceOpenDescriptions) {
    downloadMapsButton->showDescription();
    preferredSchedule->showDescription();
    removeMapsButton->showDescription();
    selectMaps->showDescription();
  }

  UIState &s = *uiState();
  UIScene &scene = s.scene;

  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  SubMaster &fpsm = *(fs.sm);

  const cereal::MapdExtendedOut::Reader &mapdExtendedOut = fpsm["mapdExtendedOut"].getMapdExtendedOut();
  const cereal::MapdDownloadProgress::Reader &downloadProgress = mapdExtendedOut.getDownloadProgress();

  bool mapDownloadActive = downloadProgress.getActive();

  int mapDownloadDownloaded = downloadProgress.getDownloadedFiles();
  int mapDownloadTotal = downloadProgress.getTotalFiles();

  hasMapsSelected = !params.get("MapsSelected").empty();

  bool parked = !scene.started || frogpilot_scene.parked || parent->isFrogsGoMoo;

  removeMapsButton->setVisible(mapsFolderPath.exists());

  if (mapDownloadActive) {
    downloadMapsButton->setText(tr("CANCEL"));
    downloadStatus->setText(tr("Calculating..."));

    downloadStatus->setVisible(true);

    lastMapsDownload->setVisible(false);
    removeMapsButton->setVisible(false);

    updateDownloadLabels(mapDownloadDownloaded, mapDownloadTotal);
  } else {
    downloadMapsButton->setEnabled(!cancellingDownload && hasMapsSelected && frogpilot_scene.online && parked);
    downloadMapsButton->setValue(frogpilot_scene.online ? (parked ? "" : tr("Not parked")) : tr("Offline..."));
  }
}


void FrogPilotMapsPanel::updateState(const UIState &s, const FrogPilotUIState &fs) {
  if (!isVisible()) {
    return;
  }

  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const UIScene &scene = s.scene;
  const SubMaster &fpsm = *(fs.sm);

  const cereal::MapdExtendedOut::Reader &mapdExtendedOut = fpsm["mapdExtendedOut"].getMapdExtendedOut();
  const cereal::MapdDownloadProgress::Reader &downloadProgress = mapdExtendedOut.getDownloadProgress();

  bool mapDownloadActive = downloadProgress.getActive();
  bool parked = !scene.started || frogpilot_scene.parked || parent->isFrogsGoMoo;

  int mapDownloadDownloaded = downloadProgress.getDownloadedFiles();
  int mapDownloadTotal = downloadProgress.getTotalFiles();

  if (mapDownloadActive && !cancellingDownload) {
    updateDownloadLabels(mapDownloadDownloaded, mapDownloadTotal);
  } else if (downloadMapsButton->text() == tr("CANCEL")) {
    updateDownloadLabels(mapDownloadDownloaded, mapDownloadTotal);
  } else {
    downloadMapsButton->setEnabled(!cancellingDownload && hasMapsSelected && frogpilot_scene.online && parked);
    downloadMapsButton->setValue(frogpilot_scene.online ? (parked ? "" : tr("Not parked")) : tr("Offline..."));
  }

  parent->keepScreenOn = mapDownloadActive;
}

void FrogPilotMapsPanel::cancelDownload() {
  cancellingDownload = true;

  downloadMapsButton->setEnabled(false);

  downloadETA->setText(tr("Calculating..."));
  downloadMapsButton->setText(tr("CANCEL"));
  downloadStatus->setText(tr("Calculating..."));
  downloadTimeElapsed->setText(tr("Calculating..."));

  params_memory.putBool("CancelDownloadMaps", true);
  params_memory.remove("DownloadMaps");

  QTimer::singleShot(2500, [this]() {
    cancellingDownload = false;

    downloadMapsButton->setEnabled(true);

    downloadMapsButton->setText(tr("DOWNLOAD"));

    downloadETA->setVisible(false);
    downloadStatus->setVisible(false);
    downloadTimeElapsed->setVisible(false);

    lastMapsDownload->setVisible(true);
    removeMapsButton->setVisible(mapsFolderPath.exists());

    update();
  });
}

void FrogPilotMapsPanel::startDownload() {
  downloadETA->setText(tr("Calculating..."));
  downloadMapsButton->setText(tr("CANCEL"));
  downloadStatus->setText(tr("Calculating..."));
  downloadTimeElapsed->setText(tr("Calculating..."));

  downloadETA->setVisible(true);
  downloadStatus->setVisible(true);
  downloadTimeElapsed->setVisible(true);

  lastMapsDownload->setVisible(false);
  removeMapsButton->setVisible(false);

  elapsedTime.start();
  startTime = QDateTime::currentDateTime();

  params_memory.putBool("DownloadMaps", true);
}

void FrogPilotMapsPanel::updateDownloadLabels(int downloadedFiles, int totalFiles) {
  if (downloadedFiles == totalFiles && totalFiles != 0) {
    downloadMapsButton->setText(tr("DOWNLOAD"));
    lastMapsDownload->setText(formatCurrentDate());

    downloadETA->setVisible(false);
    downloadStatus->setVisible(false);
    downloadTimeElapsed->setVisible(false);

    lastMapsDownload->setVisible(true);
    removeMapsButton->setVisible(true);

    params.put("LastMapsUpdate", formatCurrentDate().toStdString());

    update();

    return;
  }

  static int previousDownloadedFiles = 0;
  if (downloadedFiles != previousDownloadedFiles) {
    std::thread([this]() {
      mapsSize->setText(calculateDirectorySize(mapsFolderPath));
    }).detach();
  }

  downloadETA->setText(QString("%1").arg(formatETA(elapsedTime.elapsed(), downloadedFiles, previousDownloadedFiles, totalFiles, startTime)));
  downloadStatus->setText(QString("%1 / %2 (%3%)").arg(downloadedFiles).arg(totalFiles).arg((downloadedFiles * 100) / (totalFiles == 0 ? 1 : totalFiles)));
  downloadTimeElapsed->setText(formatElapsedTime(elapsedTime.elapsed()));

  previousDownloadedFiles = downloadedFiles;
}
