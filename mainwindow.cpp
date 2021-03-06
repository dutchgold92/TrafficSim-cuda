#include "mainwindow.h"
#include "ui_mainwindow.h"

/**
* @brief MainWindow::MainWindow Initialises the MainWindow.
* @param parent Unused parameter.
*/
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->model = new Model();
    this->updating = true;
    this->cell_size = DEFAULT_CELL_SIZE;
    this->show_road_directions = false;
    this->rendered_road_generations = new unsigned long[this->model->get_road_count()];
    memset(this->rendered_road_generations, 0, (this->model->get_road_count() * sizeof(long)));
    this->scene = new QGraphicsScene(0, 0, ui->gfx->frameSize().width(), ui->gfx->frameSize().height());
    this->scene->setBackgroundBrush(QBrush(Qt::black));
    ui->gfx->addAction(ui->actionPlotInputDensity);
    ui->gfx->addAction(ui->actionPlotInputAndOverallDensity);
    ui->gfx->addAction(ui->actionPlotTrafficThroughput);
    ui->gfx->addAction(ui->actionPlotComputationTime);
    ui->gfx->setScene(this->scene);
    ui->gfx->show();
    this->follow_vehicle_road = -1;
    this->follow_vehicle_cell = -1;
    this->plot_widget = 0;
    this->plot_time_steps = DEFAULT_PLOT_TIME_STEPS;
    ui->closePlotButton->hide();

    QString updateValueString;
    updateValueString.append("(");
    updateValueString.append(QString::number(DEFAULT_USLEEP_INTERVAL));
    updateValueString.append(" ms)");
    ui->updateIntervalValueLabel->setText(updateValueString);

    QString densityValueString;
    densityValueString.append("(");
    densityValueString.append(QString::number(DEFAULT_INITIAL_DENSITY));
    densityValueString.append(")");
    ui->densityInputValueLabel->setText(densityValueString);
    ui->densityInput->setValue(DEFAULT_DESIRED_DENSITY * 100);

    switch(this->model->get_road_directions()[0])
    {
        case Model::Up:
            ui->gfx->setAlignment(Qt::AlignBottom);
            break;
        case Model::Down:
            ui->gfx->setAlignment(Qt::AlignTop);
            break;
        case Model::Left:
            ui->gfx->setAlignment(Qt::AlignRight);
            break;
        case Model::Right:
            ui->gfx->setAlignment(Qt::AlignLeft);
            break;
    }

    connect(this, SIGNAL(model_updated()), this, SLOT(draw_model()), Qt::QueuedConnection);
    connect(this, SIGNAL(model_updated()), this, SLOT(plot()), Qt::QueuedConnection);
    connect(this->scene, SIGNAL(selectionChanged()), this, SLOT(scene_selection()), Qt::QueuedConnection);
    this->draw_model();
    this->model_updater = new ModelUpdater(this, this->model);
    this->model_updater->start();
}

/**
* @brief MainWindow::~MainWindow Destroys the MainWindow.
*/
MainWindow::~MainWindow()
{
    delete this->model;
    delete ui;
}

/**
* @brief MainWindow::draw_model Draws the model road network unto the GUI.
*/
void MainWindow::draw_model()
{
    this->scene->clear();
    this->follow_vehicle_road = cuda_get_follow_vehicle_road();
    this->follow_vehicle_cell = cuda_get_follow_vehicle_cell();
    this->draw_road(0, true, 0, 0);
    this->scene->setSceneRect(this->scene->itemsBoundingRect());
}

/**
 * @brief MainWindow::draw_road Draws the specific road unto the GUI.
 * @param road_index Index of road.
 * @param process_forward Direction of processing.
 * @param x Coordinate on the x-axis.
 * @param y Coordinate on the y-axis.
 */
void MainWindow::draw_road(unsigned int road_index, bool process_forward, qreal x, qreal y)
{
    qreal start_x = x;
    qreal start_y = y;

    if(this->rendered_road_generations[road_index] > this->model->get_generation())
        return;

    Model::Direction direction = this->model->get_road_directions()[road_index];

    if(process_forward)
    {
        for(unsigned int i = 0; i < this->model->get_road_lengths()[road_index]; i++)
        {
            switch(direction)
            {
                case Model::Up:
                    y -= this->cell_size;
                    break;
                case Model::Down:
                    y += this->cell_size;
                    break;
                case Model::Left:
                    x -= this->cell_size;
                    break;
                case Model::Right:
                    x += this->cell_size;
                    break;
            }

            this->draw_cell(road_index, i, x, y, this->cell_size, this->cell_size, direction);
        }
    }
    else
    {
        for(unsigned int i = this->model->get_road_lengths()[road_index]; i-->0;)
        {
            switch(direction)
            {
                case Model::Up:
                    y += this->cell_size;
                    break;
                case Model::Down:
                    y -= this->cell_size;
                    break;
                case Model::Left:
                    x += this->cell_size;
                    break;
                case Model::Right:
                    x -= this->cell_size;
                    break;
            }

            this->draw_cell(road_index, i, x, y, this->cell_size, this->cell_size, direction);
        }
    }

    if(process_forward)
    {
        switch(direction)
        {
            case Model::Up:
                y -= this->cell_size;
                break;
            case Model::Down:
                y += this->cell_size;
                break;
            case Model::Left:
                x -= this->cell_size;
                break;
            case Model::Right:
                x += this->cell_size;
                break;
        }
    }
    else
    {
        switch(direction)
        {
            case Model::Up:
                y += this->cell_size;
                break;
            case Model::Down:
                y -= this->cell_size;
                break;
            case Model::Left:
                x += this->cell_size;
                break;
            case Model::Right:
                x -= this->cell_size;
                break;
        }
    }

    this->rendered_road_generations[road_index]++;

    for(unsigned int i = 0; i < this->model->get_road_link_count(); i++)
    {
        for(unsigned int j = 0; j < this->model->get_road_links()[i].origin_road_count; j++)
        {
            if(this->model->get_road_links()[i].origin_roads[j] == road_index)
            {
                for(unsigned int k = 0; k < this->model->get_road_links()[i].destination_road_count; k++)
                {
                    if(this->rendered_road_generations[this->model->get_road_links()[i].destination_roads[k]] <= this->model->get_generation())
                    {
                        if(process_forward)
                            this->draw_road(this->model->get_road_links()[i].destination_roads[k], true, x, y);
                        else
                            this->draw_road(this->model->get_road_links()[i].destination_roads[k], true, start_x, start_y);
                    }
                }

                break;
            }
        }

        for(unsigned int j = 0; j < this->model->get_road_links()[i].destination_road_count; j++)
        {
            if(this->model->get_road_links()[i].destination_roads[j] == road_index)
            {
                for(unsigned int k = 0; k < this->model->get_road_links()[i].origin_road_count; k++)
                {
                    if(this->rendered_road_generations[this->model->get_road_links()[i].origin_roads[k]] <= this->model->get_generation())
                    {
                        if(!process_forward)
                            this->draw_road(this->model->get_road_links()[i].origin_roads[k], false, x, y);
                        else
                            this->draw_road(this->model->get_road_links()[i].origin_roads[k], false, start_x, start_y);
                    }
                }

                break;
            }
        }
    }
}

/**
 * @brief MainWindow::draw_cell Draws the specified cell unto the GUI.
 * @param road_index Index of road.
 * @param cell_index Index of cell.
 * @param x Coordinate on the x-axis.
 * @param y Coordinate on the y-axis.
 * @param cell_width Width of cell to be drawn.
 * @param cell_height Height of cell to be drawn.
 * @param direction Orientation of drawn cell.
 */
void MainWindow::draw_cell(unsigned int road_index, unsigned int cell_index, qreal x, qreal y, qreal cell_width, qreal cell_height, Model::Direction direction)
{
    bool is_empty = (this->model->get_cells()[road_index][cell_index] < 0);
    GraphicsCellItem *cell = new GraphicsCellItem(is_empty, road_index, cell_index, x, y, cell_width, cell_height);
    this->scene->addItem(cell);

    if(this->follow_vehicle_road == road_index && this->follow_vehicle_cell == cell_index)
        cell->setSelected(true);

    if(this->show_road_directions && (cell_index == (this->model->get_road_lengths()[road_index] / 2)))
        this->draw_directional_arrow(x, y, direction);
}

/**
 * @brief MainWindow::on_playPauseButton_pressed Reacts to Play/Pause actions from the user.
 */
void MainWindow::on_playPauseButton_pressed()
{
    if(this->updating)
    {
        this->model_updater->stop();
        ui->playPauseButton->setText("Play");
        ui->stepButton->setEnabled(true);
    }
    else
    {
        this->model_updater->start();
        ui->playPauseButton->setText("Pause");
        ui->stepButton->setDisabled(true);
    }

    this->updating = !this->updating;
}

/**
 * @brief MainWindow::on_stepButton_pressed Evolves the model for a single generation.
 */
void MainWindow::on_stepButton_pressed()
{
    this->model->update();
    emit(model_updated());
}

/**
 * @brief MainWindow::on_displayScaleInput_valueChanged Scales the display by input value.
 */
void MainWindow::on_displayScaleInput_valueChanged(int value)
{
    this->cell_size = value;
}

/**
 * @brief MainWindow::on_showRoadDirectionsInput_toggled Sets this->show_road_directions according to input value.
 */
void MainWindow::on_showRoadDirectionsInput_toggled(bool checked)
{
    this->show_road_directions = checked;
}

/**
 * @brief MainWindow::draw_directional_arrow Draws a directional arrow next to a road on the GUI.
 * @param x Coordinate on the x-axis.
 * @param y Coordinate on the y-axis.
 * @param direction Direction of arrow to be drawn.
 */
void MainWindow::draw_directional_arrow(qreal x, qreal y, Model::Direction direction)
{
    qreal arrow_x = x;
    qreal arrow_y = y;
    QString arrow_text;

    switch(direction)
    {
        case Model::Right:
            arrow_y += this->cell_size;
            arrow_text = QString::fromUtf8("→");
            break;
        case Model::Left:
            arrow_y += this->cell_size;
            arrow_text = QString::fromUtf8("←");
            break;
        case Model::Down:
            arrow_x += this->cell_size;
            arrow_text = QString::fromUtf8("↓");
            break;
        case Model::Up:
            arrow_x += this->cell_size;
            arrow_text = QString::fromUtf8("↑");
            break;
    }

    QGraphicsSimpleTextItem *arrow = new QGraphicsSimpleTextItem(0);
    arrow->setBrush(QBrush(Qt::white));
    arrow->setText(arrow_text);
    arrow->setX(arrow_x);
    arrow->setY(arrow_y);
    scene->addItem(arrow);
}

/**
 * @brief MainWindow::on_updateIntervalInput_valueChanged Reacts to user input by changing the usleep interval.
 */
void MainWindow::on_updateIntervalInput_valueChanged(int value)
{
    QString string;
    string.append("(");
    string.append(QString::number(value));
    string.append(" ms)");
    ui->updateIntervalValueLabel->setText(string);
    this->model_updater->set_usleep_interval((float)ui->updateIntervalInput->value() / 100);
}

/**
 * @brief MainWindow::on_densityInput_valueChanged Reacts to user input by changing the desired traffic density.
 */
void MainWindow::on_densityInput_valueChanged(int value)
{
    QString string;
    string.append("(");
    string.append(QString::number(float(value) / 100));
    string.append(")");
    ui->densityInputValueLabel->setText(string);
    this->model->set_desired_density(((float)ui->densityInput->value()) / 100);
}

/**
 * @brief MainWindow::scene_selection Reacts to user selection, enabling/disabling vehicle tracking.
 */
void MainWindow::scene_selection()
{
    if(this->scene->selectedItems().isEmpty())
        return;
    else
    {
        GraphicsCellItem *selected = qgraphicsitem_cast<GraphicsCellItem*>(this->scene->selectedItems().back());
        selected->setSelected(true);
        cuda_set_follow_vehicle(selected->get_road(), selected->get_cell());
    }
}

/**
 * @brief MainWindow::on_realisticTrafficSynthesisInput_toggled Sets the traffic synthesis
 * method according to user selection.
 */
void MainWindow::on_realisticTrafficSynthesisInput_toggled(bool checked)
{
    this->model->set_realistic_traffic_synthesis(checked);
}

/**
* @brief MainWindow::plot Plots model data.
*/
void MainWindow::plot()
{
    if(this->plot_widget != 0)
    {
        switch(this->plot_type)
        {
            case input_density:
                this->plot_data_y.pop_front();
                this->plot_data_y.push_back(this->model->get_input_density());
                this->plot_widget->graph(0)->setData(this->plot_data_x, this->plot_data_y);
                break;
            case overall_density_vs_input_density:
                this->plot_data_y.pop_front();
                this->plot_data_y2.pop_front();
                this->plot_data_y.push_back(this->model->get_input_density());
                this->plot_data_y2.push_back(this->model->get_model_density());
                this->plot_widget->graph(0)->setData(this->plot_data_x, this->plot_data_y);
                this->plot_widget->graph(1)->setData(this->plot_data_x, this->plot_data_y2);
                this->plot_widget->graph(1)->setPen(QPen(Qt::red));
                break;
            case traffic_throughput:
                this->plot_data_y.pop_front();
                this->plot_data_y.push_back(this->model->get_vehicles_out_last_generation());
                this->plot_widget->graph(0)->setData(this->plot_data_x, this->plot_data_y);
                break;
            case compute_time:
                this->plot_data_y.pop_front();
                this->plot_data_y.push_back(this->model->get_last_evolution_time());
                this->plot_widget->graph(0)->setData(this->plot_data_x, this->plot_data_y);
                break;
        }

        this->plot_widget->replot();
    }
}

/**
* @brief MainWindow::on_actionPlotInputDensity_triggered Opens the plotting pane to plot input density,
* based on user selection.
*/
void MainWindow::on_actionPlotInputDensity_triggered()
{
    if(this->plot_widget != 0)
        this->on_closePlotButton_pressed();

    this->plot_type = input_density;
    ui->plotLayout->removeWidget(this->plot_widget);
    ui->plotLayout->addWidget(this->plot_widget = new QCustomPlot(this->plot_widget));
    this->plot_widget->xAxis->setLabel("Time Steps");
    this->plot_widget->xAxis->setRange(-this->plot_time_steps, 0);
    this->plot_widget->yAxis->setLabel("Input Traffic Density");
    this->plot_widget->yAxis->setRange(0, 1);
    this->plot_widget->setMinimumWidth(this->frameSize().width() / 2);
    this->plot_widget->setMinimumHeight(this->frameSize().height() / 2);
    this->plot_data_x.clear();
    this->plot_data_y.clear();
    this->plot_data_y2.clear();

    for(signed int i = 0; i < this->plot_time_steps; i++)
    {
        this->plot_data_x.push_back(i - this->plot_time_steps);
        this->plot_data_y.push_back(0);
    }

    this->plot_widget->addGraph();
    this->plot();
    this->plot_widget->show();
    ui->closePlotButton->show();
}

/**
* @brief MainWindow::on_actionPlotInputAndOverallDensity_triggered Opens the plotting pane to plot input density
* and overall density, based on user selection.
*/
void MainWindow::on_actionPlotInputAndOverallDensity_triggered()
{
    if(this->plot_widget != 0)
        this->on_closePlotButton_pressed();

    this->plot_type = overall_density_vs_input_density;
    ui->plotLayout->removeWidget(this->plot_widget);
    ui->plotLayout->addWidget(this->plot_widget = new QCustomPlot(this->plot_widget));
    this->plot_widget->xAxis->setLabel("Time Steps");
    this->plot_widget->xAxis->setRange(-this->plot_time_steps, 0);
    this->plot_widget->yAxis->setLabel("Input (blue)/Overall (red) Traffic Density");
    this->plot_widget->yAxis->setRange(0, 1);
    this->plot_widget->setMinimumWidth(this->frameSize().width() / 2);
    this->plot_widget->setMinimumHeight(this->frameSize().height() / 2);
    this->plot_data_x.clear();
    this->plot_data_y.clear();
    this->plot_data_y2.clear();

    for(signed int i = 0; i < this->plot_time_steps; i++)
    {
        this->plot_data_x.push_back(i - this->plot_time_steps);
        this->plot_data_y.push_back(0);
        this->plot_data_y2.push_back(0);
    }

    this->plot_widget->addGraph();
    this->plot_widget->addGraph();
    this->plot();
    this->plot_widget->show();
    ui->closePlotButton->show();
}

/**
* @brief MainWindow::on_closePlotButton_pressed Closes the plotting pane.
*/
void MainWindow::on_closePlotButton_pressed()
{
    this->plot_widget->hide();
    this->plot_widget = 0;
    this->plot_data_x.clear();
    this->plot_data_y.clear();
    this->plot_data_y2.clear();
    ui->closePlotButton->hide();
}

/**
* @brief MainWindow::resizeEvent Reacts to user resizing of the window.
*/
void MainWindow::resizeEvent(QResizeEvent *)
{
    if(this->plot_widget != 0 && !this->plot_widget->isHidden())
    {
        this->plot_widget->setMinimumWidth(this->frameSize().width() / 2);
        this->plot_widget->setMinimumHeight(this->frameSize().height() / 2);
    }
}

/**
 * @brief MainWindow::on_actionPlotTrafficThroughput_triggered Plots traffic throughput in the model.
 */
void MainWindow::on_actionPlotTrafficThroughput_triggered()
{
    if(this->plot_widget != 0)
        this->on_closePlotButton_pressed();

    this->plot_type = traffic_throughput;
    ui->plotLayout->removeWidget(this->plot_widget);
    ui->plotLayout->addWidget(this->plot_widget = new QCustomPlot(this->plot_widget));
    this->plot_widget->xAxis->setLabel("Time Steps");
    this->plot_widget->xAxis->setRange(-20, 0);
    this->plot_widget->yAxis->setLabel("Traffic Throughput (Vehicles)");
    this->plot_widget->yAxis->setRange(0, (this->model->get_output_road_count() * 2));
    this->plot_widget->setMinimumWidth(this->frameSize().width() / 2);
    this->plot_widget->setMinimumHeight(this->frameSize().height() / 2);
    this->plot_data_x.clear();
    this->plot_data_y.clear();
    this->plot_data_y2.clear();

    for(signed int i = 0; i < this->plot_time_steps; i++)
    {
        this->plot_data_x.push_back(i - this->plot_time_steps);
        this->plot_data_y.push_back(0);
    }

    this->plot_widget->addGraph();
    this->plot();
    this->plot_widget->show();
    ui->closePlotButton->show();
}

/**
 * @brief MainWindow::on_actionPlotComputationTime_triggered Plots time taken to compute model
 * in each of the last 20 generations.
 */
void MainWindow::on_actionPlotComputationTime_triggered()
{
    if(this->plot_widget != 0)
        this->on_closePlotButton_pressed();

    this->plot_type = compute_time;
    ui->plotLayout->removeWidget(this->plot_widget);
    ui->plotLayout->addWidget(this->plot_widget = new QCustomPlot(this->plot_widget));
    this->plot_widget->xAxis->setLabel("Model Generation");
    this->plot_widget->xAxis->setRange(-20, 0);
    this->plot_widget->yAxis->setLabel("Computation Time (ms)");
    this->plot_widget->yAxis->setRange(0, (this->model->get_last_evolution_time() * 10));
    this->plot_widget->setMinimumWidth(this->frameSize().width() / 2);
    this->plot_widget->setMinimumHeight(this->frameSize().height() / 2);
    this->plot_data_x.clear();
    this->plot_data_y.clear();
    this->plot_data_y2.clear();

    for(signed int i = 0; i < 20; i++)
    {
        this->plot_data_x.push_back(i - 20);
        this->plot_data_y.push_back(0);
    }

    this->plot_widget->addGraph();
    this->plot();
    this->plot_widget->show();
    ui->closePlotButton->show();
}
