#ifndef MODELUPDATER_H
#define MODELUPDATER_H

#define DEFAULT_USLEEP_INTERVAL 0.5

#include <QThread>
#include <model.h>

class ModelUpdater : public QThread
{
    Q_OBJECT
public:
    explicit ModelUpdater(QObject *parent = 0, Model *model = 0);
    void set_usleep_interval(float usleep_interval);
    void stop();
private:
    Model *model;
    bool stopped;
    double usleep_interval;
    void run();
    void run_once();
signals:
    void model_updated();
};

#endif // MODELUPDATER_H
