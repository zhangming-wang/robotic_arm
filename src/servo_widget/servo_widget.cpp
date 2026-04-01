#include "servo_widget.h"
#include "./ui_servo_widget.h"

ServoWidget::ServoWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::ServoWidget), status_timer_(new QTimer(this)) {
    ui->setupUi(this);
    // ui->label_status->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    ServoManager::instance();
    _init_comboBox();
    _init_tableWidget();

    connect(ui->comboBox_servo, &QComboBox::currentTextChanged, this, &ServoWidget::on_refresh_current_servo);
    connect(ui->pushButton_refresh, &QPushButton::clicked, this, &ServoWidget::on_refresh_current_servo);
    connect(ui->pushButton_ping, &QPushButton::clicked, this, &ServoWidget::on_ping_current_servo);
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
    QVector<QString> addr_info = {"首地址", "功能", "默认值", "存储区", "权限", "最小值", "最大值", "单位", "当前值", "读取", "写入"};

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
        doubleSpinBox->setRange(-999999.0, 999999.0); // 设置范围
        doubleSpinBox->setDecimals(decimals);         // 设置小数位数
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
                QMessageBox::critical(this, "错误", "读取失败");
                doubleSpinBox->setValue(-1);
            } else {
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
                QMessageBox::critical(this, "错误", "写入失败");
            }
        });

        row++;
    }

    ui->tableWidget_servo->verticalHeader()->hide();
    ui->tableWidget_servo->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidget_servo->horizontalHeader()->setSectionResizeMode(8, QHeaderView::Fixed);

    ui->tableWidget_servo->horizontalHeader()->resizeSection(8, 150);
}

void ServoWidget::on_refresh_current_servo() {
    int should_read_size = std::prev(servo_param_addr_map_.end())->first - servo_param_addr_map_.begin()->first + std::prev(servo_status_addr_map_.end())->second.size;
    std::vector<uint8_t> read_buffer(should_read_size);
    int actual_read_size = ServoManager::instance().read_addr(ui->comboBox_servo->currentText().toInt(), servo_param_addr_map_.begin()->first, read_buffer.data(), should_read_size);

    bool read_success = true;
    if (actual_read_size != should_read_size) {
        QMessageBox::critical(this, "错误", "读取当前舵机参数失败");
        read_success = false;
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
        QMessageBox::information(this, "通知", "当前舵机在线");
    } else {
        QMessageBox::critical(this, "错误", "当前舵机离线");
    }
}

void ServoWidget::on_update_servo_status() {
    int should_read_size = std::prev(servo_status_addr_map_.end())->first - servo_status_addr_map_.begin()->first + std::prev(servo_status_addr_map_.end())->second.size;
    std::vector<uint8_t> read_buffer(should_read_size);
    int actual_read_size = ServoManager::instance().read_addr(ui->comboBox_servo->currentText().toInt(), servo_status_addr_map_.begin()->first, read_buffer.data(), should_read_size);

    bool read_success = true;
    if (actual_read_size != should_read_size) {
        read_success = false;
    }

    QString status_info;
    int row = 0;
    for (auto it = servo_status_addr_map_.begin(); it != servo_status_addr_map_.end(); it++) {
        int value = -1;
        int decimals = (fabs(fabs(it->second.factor) - 1.0) > pow(1, -6)) ? 2 : 0;

        if (read_success) {
            if (it->second.size == 1) {
                value = read_buffer[it->first - servo_status_addr_map_.begin()->first];
            } else if (it->second.size == 2) {
                value = ServoManager::instance().merge_two_byte(read_buffer[it->first - servo_status_addr_map_.begin()->first], read_buffer[it->first - servo_status_addr_map_.begin()->first + 1], it->second.sign_bit);
            }
        }

        status_info += QString("<td>"
                               "<td align='left'>%1:</td>"             // 描述右对齐
                               "<td align='center' width='40'>%2</td>" // 数值居中
                               "<td align='left'>%3</td>"              // 单位左对齐
                               "</td>")
                           .arg(QString::fromStdString(it->second.description))
                           .arg(QString::number(value * it->second.factor, 'f', decimals))
                           .arg(QString::fromStdString(it->second.unit) + ", ");
        row++;
    }

    ui->label_status->setText(status_info);
}