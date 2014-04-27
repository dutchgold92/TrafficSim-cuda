#include "modelupdater.h"

/**
* @brief ModelUpdater::ModelUpdater Initialises the ModelUpdater object.
* @param parent Parent object - MainWindow expected.
* @param model Model object to manipulate.
*/
ModelUpdater::ModelUpdater(QObject *parent, Model *model) : QThread(parent)
{
    this->usleep_interval = DEFAULT_USLEEP_INTERVAL;
    this->model = model;
    this->stopped = false;
    connect(this, SIGNAL(model_updated()), parent, SLOT(draw_model()), Qt::QueuedConnection);
    connect(this, SIGNAL(model_updated()), parent, SLOT(plot()), Qt::QueuedConnection);
}

/**
* @brief ModelUpdater::run Starts the update thread.
*/
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

/**
* @brief ModelUpdater::set_update_interval Changes the object's defined update interval.
*/
void ModelUpdater::set_usleep_interval(float usleep_interval)
{
    this->usleep_interval = usleep_interval;
}

/**
* @brief ModelUpdater::stop Stops the thread.
*/
void ModelUpdater::stop()
{
    this->stopped = true;
}
