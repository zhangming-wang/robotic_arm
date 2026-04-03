#include "main_window/main_window.h"
#include <QApplication>
#include <iostream>

int main(int argc, char **argv) {
    QApplication a(argc, argv);

    MainWindow w;
    w.showMaximized();

    a.exec();
}