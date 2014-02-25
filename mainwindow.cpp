#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Model *m = new Model();

    for(int i = 0; i < 1000; i++)
    {
        //m->display();
        m->update();
    }

    cout << "done" << endl;
}

MainWindow::~MainWindow()
{
    delete ui;
}
