#pragma once

#include "servo_widget/servo_widget.h"
#include "ui_main_window.h"
#include <QCloseEvent>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QMdiSubWindow>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void on_node_closed_sig();

  protected:
    void closeEvent(QCloseEvent *event) override;

  private:
    std::shared_ptr<Ui::MainWindow> ui;

    ServoWidget *servo_widget_{nullptr};

    const QString CONFIG_FILE_PATH = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/" + "." + QCoreApplication::applicationName() + "/settings.json";

    // void _readConfigJson();
    // void _writeConfigJson();
};