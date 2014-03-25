#include "mainwindow.h"
#include "ui_mainwindow.h"

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
    ui->gfx->setScene(this->scene);
    ui->gfx->show();
    ui->closePlotButton->hide();
    this->follow_vehicle_road = -1;
    this->follow_vehicle_cell = -1;

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
    connect(this->scene, SIGNAL(selectionChanged()), this, SLOT(scene_selection()), Qt::QueuedConnection);
    this->draw_model();
    this->model_updater = new ModelUpdater(this, this->model);
    this->model_updater->start();
}

MainWindow::~MainWindow()
{
    delete this->model;
    delete ui;
}

void MainWindow::draw_model()
{
    this->scene->clear();
    this->follow_vehicle_road = cuda_get_follow_vehicle_road();
    this->follow_vehicle_cell = cuda_get_follow_vehicle_cell();
    this->draw_road(0, true, 0, 0);
    this->scene->setSceneRect(this->scene->itemsBoundingRect());
}

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

void MainWindow::on_stepButton_pressed()
{
    this->model->update();
    emit(model_updated());
}

void MainWindow::on_displayScaleInput_valueChanged(int value)
{
    this->cell_size = value;
}

void MainWindow::on_showRoadDirectionsInput_toggled(bool checked)
{
    this->show_road_directions = checked;
}

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

void MainWindow::on_updateIntervalInput_valueChanged(int value)
{
    QString string;
    string.append("(");
    string.append(QString::number(value));
    string.append(" ms)");
    ui->updateIntervalValueLabel->setText(string);
    this->model_updater->set_usleep_interval((float)ui->updateIntervalInput->value() / 100);
}

void MainWindow::on_densityInput_valueChanged(int value)
{
    QString string;
    string.append("(");
    string.append(QString::number(float(value) / 100));
    string.append(")");
    ui->densityInputValueLabel->setText(string);
    this->model->set_desired_density(((float)ui->densityInput->value()) / 100);
}

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
