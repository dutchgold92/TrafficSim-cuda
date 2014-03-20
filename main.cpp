#include "mainwindow.h"
#include <QApplication>
#include <QElapsedTimer>
#include "model.h"

void benchmark()
{
    Model *m = new Model();

    QElapsedTimer timer;
    qint64 m_seconds;
    timer.start();

    for(int i = 0; i < 10; i++)
    {
//        m->display();
        m->update();
    }

    m_seconds = timer.elapsed();

    cout << "finished after: " << m_seconds << "ms" << endl;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();

//    benchmark();
}
