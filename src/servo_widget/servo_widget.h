#ifndef SERVOWIDGET_H
#define SERVOWIDGET_H

#include "servo_manager/servo_manager.h"
#include <QDebug>
#include <QHideEvent>
#include <QMessageBox>
#include <QPushButton>
#include <QShowEvent>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>
#include <map>
#include <math.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class ServoWidget;
}
QT_END_NAMESPACE

class ServoWidget : public QWidget {
    Q_OBJECT

    struct ServoAddrInfo {
        uint8_t size;            // 数据内存大小
        bool is_eprom;           // 是否是eprom
        bool only_read;          // 是否只读
        uint8_t sign_bit;        // 负数位，0表示无符号数
        int default_value;       // 默认值
        int min_value;           // 最小值
        int max_value;           // 最大值
        float factor;            // 修正系数
        std::string description; // 描述
        std::string unit;        // 单位
    };

  public:
    ServoWidget(QWidget *parent = nullptr);
    ~ServoWidget();

    void on_refresh_current_servo();
    void on_ping_current_servo();
    void on_update_servo_status();

  protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

  private:
    void _init_tableWidget();
    void _init_comboBox();

    Ui::ServoWidget *ui;

    QTimer *status_timer_ = nullptr;

    std::vector<int> servo_vector_ = {1, 2, 3, 4, 5, 6};

    std::map<int, ServoAddrInfo> servo_param_addr_map_ = {
        {0, ServoAddrInfo{1, true, true, 0, 3, -1, -1, 1.0f, "固件主版本号", ""}},
        {1, ServoAddrInfo{1, true, true, 0, 9, -1, -1, 1.0f, "固件次版本号", ""}},
        {2, ServoAddrInfo{1, true, true, 0, 0, -1, -1, 1.0f, "END", ""}},
        {3, ServoAddrInfo{1, true, true, 0, 9, -1, -1, 1.0f, "舵机主版本号", ""}},
        {4, ServoAddrInfo{1, true, true, 0, -1, -1, -1, 1.0f, "舵机次版本号", ""}},

        {5, ServoAddrInfo{1, true, false, 0, 1, 0, 253, 1.0f, "ID", "号"}},
        {6, ServoAddrInfo{1, true, false, 0, 0, 0, 7, 1.0f, "波特率", ""}},
        {7, ServoAddrInfo{1, true, false, 0, 0, 0, 254, 2.0f, "返回延时", "us"}},
        {8, ServoAddrInfo{1, true, false, 0, 1, 0, 1, 1.0f, "应答状态级别", ""}},
        {9, ServoAddrInfo{2, true, false, 0, 0, 0, 4094, 1.0f, "最小角度限制", "步"}},

        {11, ServoAddrInfo{2, true, false, 0, 4095, 1, 4095, 1.0f, "最大角度限制", "步"}},
        {13, ServoAddrInfo{1, true, false, 0, 70, 0, 100, 1.0f, "最高温度上限", "°C"}},
        {14, ServoAddrInfo{1, true, false, 0, 80, 0, 254, 0.1f, "最高输入电压", "v"}},
        {15, ServoAddrInfo{1, true, false, 0, 40, 0, 254, 0.1f, "最低输入电压", "v"}},
        {16, ServoAddrInfo{2, true, false, 0, 1000, 0, 1000, 0.1f, "最大扭矩", "%"}},
        {18, ServoAddrInfo{1, true, false, 0, 12, 0, 254, 1.0f, "相位", ""}},
        {19, ServoAddrInfo{1, true, false, 0, 44, 0, 254, 1.0f, "卸载条件", ""}},
        {20, ServoAddrInfo{1, true, false, 0, 47, 0, 254, 1.0f, "LED报警条件", ""}},

        {21, ServoAddrInfo{1, true, false, 0, 32, 0, 254, 1.0f, "位置环P", ""}},
        {22, ServoAddrInfo{1, true, false, 0, 32, 0, 254, 1.0f, "位置环D", ""}},
        {23, ServoAddrInfo{1, true, false, 0, 0, 0, 254, 1.0f, "位置环I", ""}},
        {24, ServoAddrInfo{1, true, false, 0, 16, 0, 254, 0.1f, "最小启动力", "%"}},
        {25, ServoAddrInfo{1, true, false, 0, 0, 0, 254, 1.0f, "积分限制值", ""}},
        {26, ServoAddrInfo{1, true, false, 0, 1, 0, 32, 1.0f, "顺时针不灵敏区", "步"}},
        {27, ServoAddrInfo{1, true, false, 0, 1, 0, 32, 1.0f, "逆时针不灵敏区", "步"}},
        {28, ServoAddrInfo{2, true, false, 0, 500, 0, 511, 6.5f, "保护电流", "mA"}},

        {30, ServoAddrInfo{1, true, false, 0, 1, 1, 3, 1.0f, "角度分辨率", ""}},
        {31, ServoAddrInfo{2, true, false, 11, 0, -2047, 2047, 1.0f, "位置校正", "步"}},
        {33, ServoAddrInfo{1, true, false, 0, 0, 0, 3, 1.0f, "运行模式", ""}},
        {34, ServoAddrInfo{1, true, false, 0, 20, 0, 100, 1.0f, "保护扭矩", "%"}},
        {35, ServoAddrInfo{1, true, false, 0, 200, 0, 254, 10.0f, "保护时间", "ms"}},
        {36, ServoAddrInfo{1, true, false, 0, 80, 0, 100, 1.0f, "过载扭矩", "%"}},
        {37, ServoAddrInfo{1, true, false, 0, 10, 0, 254, 1.0f, "速度环P", ""}},
        {38, ServoAddrInfo{1, true, false, 0, 200, 0, 254, 10.0f, "过流保护时间", "ms"}},
        {39, ServoAddrInfo{1, true, false, 0, 200, 0, 254, 0.1f, "速度环I", ""}},

        {40, ServoAddrInfo{1, false, false, 0, 0, 0, 128, 1.0f, "扭矩开关", ""}},
        {41, ServoAddrInfo{1, false, false, 0, 0, 0, 254, 100.0f, "加速度", "步/s^2"}},
        {42, ServoAddrInfo{2, false, false, 15, 0, -32766, 32766, 1.0f, "目标位置", "步"}},
        {44, ServoAddrInfo{2, false, false, 10, 0, 0, 1000, 0.1f, "运行时间", "%"}},
        {46, ServoAddrInfo{2, false, false, 15, 0, -32766, 32766, 1.0f, "运行速度", "步/s"}},
        {48, ServoAddrInfo{2, false, false, 0, 1000, 0, 1000, 1.0f, "转矩限制", "%"}},
    };

    std::map<int, ServoAddrInfo> servo_status_addr_map_ = {
        {56, ServoAddrInfo{2, false, true, 15, -1, -1, -1, 1.0f, "位置", "步"}},
        {58, ServoAddrInfo{2, false, true, 15, -1, -1, -1, 1.0f, "速度", "步/s"}},
        {60, ServoAddrInfo{2, false, true, 10, -1, -1, -1, 0.1f, "负载", "%"}},
        {62, ServoAddrInfo{1, false, true, 0, -1, -1, -1, 0.1f, "电压", "V"}},
        {63, ServoAddrInfo{1, false, true, 0, -1, -1, -1, 1.0f, "温度", "°C"}},
        // {64, ServoAddrInfo{1, false, true, 0, 0, -1, -1, 1.0f, "异步写标志", ""}},
        {65, ServoAddrInfo{1, false, true, 0, 0, -1, -1, 1.0f, "状态", ""}},
        {66, ServoAddrInfo{1, false, true, 0, 0, -1, -1, 1.0f, "移动", ""}},
        // {67, ServoAddrInfo{2, false, true, 0, 0, -1, -1, 1.0f, "无定义", ""}},
        {69, ServoAddrInfo{2, false, true, 0, -1, -1, -1, 6.5f, "电流", "mA"}},
    };
};
#endif // SERVOWIDGET_H
