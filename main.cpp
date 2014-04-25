#include "mainwindow.h"
#include <QApplication>
#include <QElapsedTimer>
#include "model.h"

/**
 * @brief benchmark Times initialisation and processing of the model over the specified number of generations.
 */
void benchmark(unsigned int generations)
{
    Model *m = new Model();

    QElapsedTimer timer;
    qint64 m_seconds;
    timer.start();

    for(int i = 0; i < generations; i++)
    {
        m->update();
    }

    m_seconds = timer.elapsed();

    cout << "finished after: " << m_seconds << "ms" << endl;
}

/**
* @brief main Launches the application.
* @param argc Unused parameter.
* @param argv Unused parameter.
*/
int main(int argc, char *argv[])
{
//     benchmark(100);
//     return 0;

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
