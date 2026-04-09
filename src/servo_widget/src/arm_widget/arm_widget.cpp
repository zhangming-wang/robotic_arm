#include "arm_widget.h"
#include "./ui_arm_widget.h"

ArmWidget::ArmWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::ArmWidget) {
    ui->setupUi(this);
}

ArmWidget::~ArmWidget() {
    delete ui;
}

void ArmWidget::showEvent(QShowEvent *event) {
    QWidget::showEvent(event);
}

void ArmWidget::hideEvent(QHideEvent *event) {
    QWidget::hideEvent(event);
}