#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define DEFAULT_INITIAL_DENSITY 0.4
#define DEFAULT_CELL_SIZE 10

#include <QMainWindow>
#include <model.h>
#include <modelupdater.h>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <graphicscellitem.h>

extern "C"
void cuda_set_follow_vehicle(unsigned int road, unsigned int cell);

extern "C"
unsigned int cuda_get_follow_vehicle_road();

extern "C"
unsigned int cuda_get_follow_vehicle_cell();

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();    
private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    Model *model;
    ModelUpdater *model_updater;
    bool updating;
    unsigned long *rendered_road_generations;
    qreal cell_size;
    bool show_road_directions;
    signed int follow_vehicle_road;
    signed int follow_vehicle_cell;
    void draw_road(unsigned int road_index, bool process_forward, qreal x, qreal y);
    void draw_directional_arrow(qreal x, qreal y, Model::Direction direction);
    void draw_cell(unsigned int road_index, unsigned int cell_index, qreal x, qreal y, qreal cell_width, qreal cell_height, Model::Direction direction);
public slots:
    void draw_model();
private slots:
    void on_playPauseButton_pressed();
    void on_stepButton_pressed();
    void on_displayScaleInput_valueChanged(int value);
    void on_showRoadDirectionsInput_toggled(bool checked);
    void on_updateIntervalInput_valueChanged(int value);
    void on_densityInput_valueChanged(int value);
    void scene_selection();
    void on_realisticTrafficSynthesisInput_toggled(bool checked);

signals:
    void model_updated();
};

#endif // MAINWINDOW_H
