#include "modelupdater.h"

ModelUpdater::ModelUpdater(QObject *parent, Model *model) : QThread(parent)
{
    this->usleep_interval = DEFAULT_USLEEP_INTERVAL;
    this->model = model;
    this->stopped = false;
    connect(this, SIGNAL(model_updated()), parent, SLOT(draw_model()), Qt::QueuedConnection);
}

void ModelUpdater::run()
{
    for(;;)
    {
        if(this->stopped)
            break;

        this->model->update();
        emit(model_updated());
        this->usleep(this->usleep_interval * 1000000);
    }

    this->stopped = false;
}

void ModelUpdater::set_usleep_interval(float usleep_interval)
{
    this->usleep_interval = usleep_interval;
}

void ModelUpdater::stop()
{
    this->stopped = true;
}
