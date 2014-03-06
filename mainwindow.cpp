#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QElapsedTimer timer;
    qint64 m_seconds;
    timer.start();

    Model *m = new Model();

    for(int i = 0; i < 10; i++)
    {
        m->display();
        m->update();
    }

    m_seconds = timer.elapsed();

    cout << "finished after: " << m_seconds << "ms" << endl;
}

MainWindow::~MainWindow()
{
    delete ui;
}
