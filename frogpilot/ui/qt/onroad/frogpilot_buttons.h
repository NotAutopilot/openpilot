#pragma once

#include "selfdrive/ui/qt/onroad/buttons.h"

class DrivingPersonalityButton : public QPushButton {
  Q_OBJECT

public:
  explicit DrivingPersonalityButton(QWidget *parent = 0);

  void updateState(const UIState &s, const FrogPilotUIState &fs);

private:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void updateTheme();

  bool theme_updated;
  bool traffic_mode_active;

  int personality;

  Params params_memory{"", true};

  QMap<int, QPair<QPixmap, QSharedPointer<QMovie>>> icon_map;

  QMovie *currentGif;

  QPixmap currentImg;
};
