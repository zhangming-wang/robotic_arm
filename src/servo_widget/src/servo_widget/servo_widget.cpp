#include "servo_widget.h"
#include "./ui_servo_widget.h"

ServoWidget::ServoWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::ServoWidget), status_timer_(new QTimer(this)) {
    ui->setupUi(this);
    ui->splitter->setSizes({1000, 1});
    ui->label_status->setAlignment(Qt::AlignCenter);
    ui->label_status->setAlignment(Qt::AlignCenter); // 确保内容块在 Label 区域居中
    ui->label_status->setTextFormat(Qt::RichText);   // 明确指定为富文本格式

    ui->textEdit_info->setMinimumWidth(350);
    ui->textEdit_info->setReadOnly(true);

    ServoManager::instance();
    _init_comboBox();
    _init_tableWidget();

    connect(ui->comboBox_servo, &QComboBox::currentTextChanged, this, &ServoWidget::on_refresh_current_servo);
    connect(ui->pushButton_refresh, &QPushButton::clicked, this, &ServoWidget::on_refresh_current_servo);
    connect(ui->pushButton_ping, &QPushButton::clicked, this, &ServoWidget::on_ping_current_servo);
    connect(ui->pushButton_move, &QPushButton::clicked, this, &ServoWidget::on_move);
    connect(ui->pushButton_clear, &QPushButton::clicked, ui->textEdit_info, &QTextEdit::clear);
    connect(ui->pushButton_calibration_ofs, &QPushButton::clicked, this, &ServoWidget::on_calibration_ofs);
    connect(ui->checkBox_enable, &QCheckBox::toggled, this, &ServoWidget::on_enable_torque);
    connect(status_timer_, &QTimer::timeout, this, &ServoWidget::on_update_servo_status);

    QTimer::singleShot(500, [this]() { on_refresh_current_servo(); });
    status_timer_->setInterval(50);
}

ServoWidget::~ServoWidget() {
    status_timer_->stop();
    delete ui;
}

void ServoWidget::showEvent(QShowEvent *event) {
    status_timer_->start();
    QWidget::showEvent(event);
}

void ServoWidget::hideEvent(QHideEvent *event) {
    status_timer_->stop();
    QWidget::hideEvent(event);
}

void ServoWidget::_init_comboBox() {
    ui->comboBox_servo->blockSignals(true);
    ui->comboBox_servo->clear();
    for (auto id : servo_vector_) {
        ui->comboBox_servo->addItem(QString::number(id));
    }
    ui->comboBox_servo->blockSignals(false);
}

void ServoWidget::_init_tableWidget() {
    QVector<QString> addr_info = {" 首地址 ", " 功能 ", " 默认值 ", " 存储区 ", " 权限 ", " 最小值 ", " 最大值 ", " 单位 ", " 当前值 ", " 读取 ", " 写入 "};

    ui->tableWidget_servo->clear();
    ui->tableWidget_servo->setColumnCount(addr_info.size());
    ui->tableWidget_servo->setHorizontalHeaderLabels(addr_info.toList());
    ui->tableWidget_servo->setRowCount(servo_param_addr_map_.size());

    int row = 0;
    for (auto it = servo_param_addr_map_.begin(); it != servo_param_addr_map_.end(); ++it) {
        QTableWidgetItem *item = nullptr;
        QDoubleSpinBox *doubleSpinBox = nullptr;
        int decimals = (fabs(fabs(it->second.factor) - 1.0) > pow(1, -6)) ? 2 : 0;

        auto set_item = [this](int row, int col, QTableWidgetItem *item) {
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            item->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget_servo->setItem(row, col, item);
        };

        item = new QTableWidgetItem(QString::number(it->first));
        set_item(row, 0, item);

        item = new QTableWidgetItem(QString::fromStdString(it->second.description));
        set_item(row, 1, item);

        item = new QTableWidgetItem(QString::number(it->second.default_value * it->second.factor, 'f', decimals));
        set_item(row, 2, item);

        if (it->second.is_eprom) {
            item = new QTableWidgetItem(QString::fromStdString("EPROM"));
        } else {
            item = new QTableWidgetItem(QString::fromStdString("SRAM"));
        }
        set_item(row, 3, item);

        if (it->second.only_read) {
            item = new QTableWidgetItem(QString::fromStdString("只读"));
        } else {
            item = new QTableWidgetItem(QString::fromStdString("读写"));
        }
        set_item(row, 4, item);

        item = new QTableWidgetItem(QString::number(it->second.min_value * it->second.factor, 'f', decimals));
        set_item(row, 5, item);

        item = new QTableWidgetItem(QString::number(it->second.max_value * it->second.factor, 'f', decimals));
        set_item(row, 6, item);

        item = new QTableWidgetItem(QString::fromStdString(it->second.unit));
        set_item(row, 7, item);

        doubleSpinBox = new QDoubleSpinBox(this);
        doubleSpinBox->setRange(-65532, 65532); // 设置范围
        doubleSpinBox->setDecimals(decimals);   // 设置小数位数
        doubleSpinBox->setValue(-1);
        // doubleSpinBox->setButtonSymbols(QAbstractSpinBox::NoButtons); // 如果不需要右侧调节按钮可以隐藏
        ui->tableWidget_servo->setCellWidget(row, 8, doubleSpinBox);

        QPushButton *readBtn = new QPushButton("读取", this);
        ui->tableWidget_servo->setCellWidget(row, 9, readBtn);
        connect(readBtn, &QPushButton::clicked, this, [this, it, doubleSpinBox]() {
            int value = -1;
            if (it->second.size == 1) {
                value = ServoManager::instance().read_byte(ui->comboBox_servo->currentText().toInt(), it->first);
            } else {
                value = ServoManager::instance().read_word(ui->comboBox_servo->currentText().toInt(), it->first, it->second.sign_bit);
            }
            if (value == -1) {
                on_append_info("读取舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " " + QString::fromStdString(it->second.description) + " 失败", true);
                doubleSpinBox->setValue(-1);
            } else {
                on_append_info("读取舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " " + QString::fromStdString(it->second.description) + " 成功");
                doubleSpinBox->setValue(value * it->second.factor);
            }
        });

        QPushButton *writeBtn = new QPushButton("写入", this);
        ui->tableWidget_servo->setCellWidget(row, 10, writeBtn);
        connect(writeBtn, &QPushButton::clicked, this, [this, it, doubleSpinBox]() {
            bool res = false;
            if (it->second.size == 1) {
                res = ServoManager::instance().write_byte(ui->comboBox_servo->currentText().toInt(), it->first, doubleSpinBox->value() / it->second.factor, it->second.is_eprom);
            } else {
                res = ServoManager::instance().write_word(ui->comboBox_servo->currentText().toInt(), it->first, doubleSpinBox->value() / it->second.factor, it->second.is_eprom, it->second.sign_bit);
            }
            if (!res) {
                on_append_info("写入舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " " + QString::fromStdString(it->second.description) + " 失败", true);
            } else {
                on_append_info("写入舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " " + QString::fromStdString(it->second.description) + " 成功");
            }
        });

        row++;
    }

    ui->tableWidget_servo->verticalHeader()->hide();
    ui->tableWidget_servo->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidget_servo->horizontalHeader()->setSectionResizeMode(8, QHeaderView::Fixed);

    ui->tableWidget_servo->horizontalHeader()->resizeSection(8, 100);
}

void ServoWidget::on_refresh_current_servo() {
    int should_read_size = std::prev(servo_param_addr_map_.end())->first - servo_param_addr_map_.begin()->first + std::prev(servo_status_addr_map_.end())->second.size;
    std::vector<uint8_t> read_buffer(should_read_size);
    int actual_read_size = ServoManager::instance().read_addr(ui->comboBox_servo->currentText().toInt(), servo_param_addr_map_.begin()->first, read_buffer.data(), should_read_size);

    bool read_success = true;
    if (actual_read_size != should_read_size) {
        on_append_info("刷新舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " 参数失败", true);
        read_success = false;
    } else {
        on_append_info("刷新舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " 参数成功");
    }

    int row = 0;
    for (auto it = servo_param_addr_map_.begin(); it != servo_param_addr_map_.end(); it++) {
        int value = -1;
        if (read_success) {
            if (it->second.size == 1) {
                value = read_buffer[it->first];
            } else if (it->second.size == 2) {
                value = ServoManager::instance().merge_two_byte(read_buffer[it->first], read_buffer[it->first + 1], it->second.sign_bit);
            }
        }

        static_cast<QDoubleSpinBox *>(ui->tableWidget_servo->cellWidget(row, 8))->setValue(value * it->second.factor);
        row++;
    }
}

void ServoWidget::on_ping_current_servo() {
    if (ServoManager::instance().ping(ui->comboBox_servo->currentText().toInt())) {
        on_append_info("舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " 在线");
    } else {
        on_append_info("舵机 ID=" + QString::number(ui->comboBox_servo->currentText().toInt()) + " 离线");
    }
}

void ServoWidget::on_update_servo_status() {
    QString status_info = QString("<tr>"
                                  "</tr>");

    if (ui->checkBox_servo_0->isChecked() || ui->checkBox_servo_1->isChecked() || ui->checkBox_servo_2->isChecked() || ui->checkBox_servo_3->isChecked() || ui->checkBox_servo_4->isChecked() || ui->checkBox_servo_5->isChecked()) {
        std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};

        std::vector<double> position(ids.size()), speed(ids.size());
        bool success = ServoManager::instance().get_state(ids, position, speed);
        for (size_t i = 0; i < ids.size(); ++i) {
            if (success) {
                status_info += QString("<tr>"
                                       "<td align='right'>ID %1:</td>"                                     // 描述右对齐
                                       "<td align='center' width='10' style='color: black;'>位置  %2</td>" // 数值居中
                                       "<td align='left' width='40'>%3</td>"                               // 单位左对齐
                                       "<td align='center' width='60' style='color: black;'>速度  %4</td>" // 数值居中
                                       "<td align='left' width='40'>%5</td>"
                                       "</tr>")
                                   .arg(ids[i])
                                   .arg(QString::number(position[i], 'f', 2))
                                   .arg("rad")
                                   .arg(QString::number(speed[i], 'f', 2))
                                   .arg("rad/s");
            } else {
                status_info += QString("<tr>"
                                       "<td align='right'>ID %1:</td>"                                     // 描述右对齐
                                       "<td align='center' width='10' style='color: black;'>位置  %2</td>" // 数值居中
                                       "<td align='left' width='40'>%3</td>"                               // 单位左对齐
                                       "<td align='center' width='60' style='color: black;'>速度  %4</td>" // 数值居中
                                       "<td align='left' width='60'>%5</td>"
                                       "</tr>")
                                   .arg(ids[i])
                                   .arg(QString::number(-1, 'f', 0))
                                   .arg("rad")
                                   .arg(QString::number(-1, 'f', 0))
                                   .arg("rad/s");
            }
        }
    } else {
        int should_read_size = std::prev(servo_status_addr_map_.end())->first - servo_status_addr_map_.begin()->first + std::prev(servo_status_addr_map_.end())->second.size;
        std::vector<uint8_t> read_buffer(should_read_size);
        int actual_read_size = ServoManager::instance().read_addr(ui->comboBox_servo->currentText().toInt(), servo_status_addr_map_.begin()->first, read_buffer.data(), should_read_size);

        bool read_success = true;
        if (actual_read_size != should_read_size) {
            read_success = false;
        }

        int row = 0;
        for (auto it = servo_status_addr_map_.begin(); it != servo_status_addr_map_.end(); it++) {
            if (read_success) {
                int value = -1;
                int decimals = (fabs(fabs(it->second.factor) - 1.0) > pow(1, -6)) ? 2 : 0;

                if (it->second.size == 1) {
                    value = read_buffer[it->first - servo_status_addr_map_.begin()->first];
                } else if (it->second.size == 2) {
                    value = ServoManager::instance().merge_two_byte(read_buffer[it->first - servo_status_addr_map_.begin()->first], read_buffer[it->first - servo_status_addr_map_.begin()->first + 1], it->second.sign_bit);
                }
                status_info += QString("<tr>"
                                       "<td align='right'>%1:</td>"                                  // 描述右对齐
                                       "<td align='center' width='40' style='color: black;'>%2</td>" // 数值居中
                                       "<td align='left'>%3</td>"                                    // 单位左对齐
                                       "</tr>")
                                   .arg(QString::fromStdString(it->second.description))
                                   .arg(QString::number(value * it->second.factor, 'f', decimals))
                                   .arg(QString::fromStdString(it->second.unit));
            } else {
                status_info += QString("<tr>"
                                       "<td align='right'>%1:</td>"                                // 描述右对齐
                                       "<td align='center' width='40' style='color: red;'>%2</td>" // 数值居中
                                       "<td align='left'>%3</td>"                                  // 单位左对齐
                                       "</tr>")
                                   .arg(QString::fromStdString(it->second.description))
                                   .arg(QString::number(-1, 'f', 0))
                                   .arg(QString::fromStdString(it->second.unit));
            }
            row++;
        }
    }

    status_info += QString("<tr>"
                           "</tr>");

    ui->label_status->setText(status_info);
}

void ServoWidget::on_append_info(const QString &info, bool is_error) {
    QString cmd_string = " [" + QDateTime::currentDateTime().toString("HH:mm:ss.zzz") + "] " + info; // yyyy-MM-dd
    QString html_string;
    if (!is_error) {
        html_string = QString("<span style='color:green;'>%1</span>").arg(cmd_string.toHtmlEscaped());
    } else {
        html_string = QString("<span style='color:red;'>%1</span>").arg(cmd_string.toHtmlEscaped());
    }
    ui->textEdit_info->append(html_string);
    ui->textEdit_info->moveCursor(QTextCursor::End);
}

void ServoWidget::on_move() {
    std::vector<uint8_t> ids;
    std::vector<double> position, speed, acc;

    if (ui->checkBox_servo_0->isChecked()) {
        ids.push_back(1);
        position.push_back(ui->doubleSpinBox_position_servo_0->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_0->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_0->value());
    }
    if (ui->checkBox_servo_1->isChecked()) {
        ids.push_back(2);
        position.push_back(ui->doubleSpinBox_position_servo_1->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_1->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_1->value());
    }
    if (ui->checkBox_servo_2->isChecked()) {
        ids.push_back(3);
        position.push_back(ui->doubleSpinBox_position_servo_2->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_2->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_2->value());
    }
    if (ui->checkBox_servo_3->isChecked()) {
        ids.push_back(4);
        position.push_back(ui->doubleSpinBox_position_servo_3->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_3->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_3->value());
    }
    if (ui->checkBox_servo_4->isChecked()) {
        ids.push_back(5);
        position.push_back(ui->doubleSpinBox_position_servo_4->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_4->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_4->value());
    }
    if (ui->checkBox_servo_5->isChecked()) {
        ids.push_back(6);
        position.push_back(ui->doubleSpinBox_position_servo_5->value());
        speed.push_back(ui->doubleSpinBox_speed_servo_5->value());
        acc.push_back(ui->doubleSpinBox_acc_servo_5->value());
    }

    if (ids.empty()) {
        on_append_info("请至少选择一个舵机", true);
        return;
    }

    QString id_string = "写入舵机 ID=";
    for (size_t i = 0; i < ids.size(); ++i) {
        id_string += QString::number(ids[i]);
        if (i != ids.size() - 1) {
            id_string += ", ";
        }
    }

    bool success = ServoManager::instance().move(ids, position, speed, acc);
    if (success) {
        on_append_info(id_string + " 位置指令成功");
    } else {
        on_append_info(id_string + " 位置指令失败", true);
    }
}

void ServoWidget::on_calibration_ofs() {
    std::vector<uint8_t> ids;
    if (ui->checkBox_servo_0->isChecked()) {
        ids.push_back(1);
    }
    if (ui->checkBox_servo_1->isChecked()) {
        ids.push_back(2);
    }
    if (ui->checkBox_servo_2->isChecked()) {
        ids.push_back(3);
    }
    if (ui->checkBox_servo_3->isChecked()) {
        ids.push_back(4);
    }
    if (ui->checkBox_servo_4->isChecked()) {
        ids.push_back(5);
    }
    if (ui->checkBox_servo_5->isChecked()) {
        ids.push_back(6);
    }

    bool success = ServoManager::instance().calibration_ofs(ids);
    if (success) {
        on_append_info("设置中位成功");
    } else {
        on_append_info("设置中位失败", true);
    }
}

void ServoWidget::on_enable_torque(bool enable) {
    std::vector<uint8_t> ids;
    if (ui->checkBox_servo_0->isChecked()) {
        ids.push_back(1);
    }
    if (ui->checkBox_servo_1->isChecked()) {
        ids.push_back(2);
    }
    if (ui->checkBox_servo_2->isChecked()) {
        ids.push_back(3);
    }
    if (ui->checkBox_servo_3->isChecked()) {
        ids.push_back(4);
    }
    if (ui->checkBox_servo_4->isChecked()) {
        ids.push_back(5);
    }
    if (ui->checkBox_servo_5->isChecked()) {
        ids.push_back(6);
    }
    if (ids.empty()) {
        ui->checkBox_enable->blockSignals(true);
        ui->checkBox_enable->setChecked(!enable);
        ui->checkBox_enable->blockSignals(false);
        on_append_info("请至少选择一个舵机", true);
        return;
    }

    QString id_string = "舵机 ID=";
    for (size_t i = 0; i < ids.size(); ++i) {
        id_string += QString::number(ids[i]);
        if (i != ids.size() - 1) {
            id_string += ", ";
        }
    }

    if (ServoManager::instance().enable_torque(ids, enable)) {
        if (enable) {
            on_append_info(id_string + " 使能成功");
        } else {
            on_append_info(id_string + " 失能成功");
        }
    } else {
        ui->checkBox_enable->blockSignals(true);
        ui->checkBox_enable->setChecked(!enable);
        ui->checkBox_enable->blockSignals(false);
        if (enable) {
            on_append_info(id_string + " 使能失败", true);
        } else {
            on_append_info(id_string + " 失能失败", true);
        }
    }
}