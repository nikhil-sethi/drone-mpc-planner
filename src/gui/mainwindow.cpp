#include "mainwindow.h"
#include <QApplication>
#include <QPushButton>
#include <QDebug>
#include <QQmlApplicationEngine>

void MainWindow::init(int argc, char **argv) {
    thread = std::thread(&MainWindow::worker,this,argc,argv);
}


void MainWindow::worker(int argc, char **argv){
    QApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));


    app.exec();
}

void MainWindow::close() {
    //todo: give close command ...but how?
}
