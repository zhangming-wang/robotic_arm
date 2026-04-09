#ifndef ARM_WIDGET_H
#define ARM_WIDGET_H

#include "servo_manager/servo_manager.h"
#include <QDateTime>
#include <QDebug>
#include <QHideEvent>
#include <QMessageBox>
#include <QPushButton>
#include <QShowEvent>
#include <QSpinBox>
#include <QTextCursor>
#include <QTimer>
#include <QWidget>
#include <map>
#include <math.h>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui {
class ArmWidget;
}
QT_END_NAMESPACE

class ArmWidget : public QWidget {
    Q_OBJECT

  public:
    ArmWidget(QWidget *parent = nullptr);
    ~ArmWidget();

  protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

  private:
    Ui::ArmWidget *ui;
};
#endif // ARM_WIDGET_H
