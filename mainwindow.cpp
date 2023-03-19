#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _parase_rosbag_ptr = std::make_shared<parase_rosbag>();

    _parase_rosbag_ptr->process();
}

MainWindow::~MainWindow()
{
    delete ui;
}

