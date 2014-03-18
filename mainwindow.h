#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define DEFAULT_INITIAL_DENSITY 0.4
#define DEFAULT_CELL_SIZE 10

#include <QMainWindow>
#include <model.h>
#include <modelupdater.h>
#include <QElapsedTimer>
#include <QGraphicsScene>
#include <QGraphicsRectItem>

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
    void draw_road(unsigned int road_index, bool process_forward, qreal x, qreal y);
    void draw_directional_arrow(qreal x, qreal y, Model::Direction direction);
public slots:
    void draw_model();
private slots:
    void on_playPauseButton_pressed();
    void on_stepButton_pressed();
    void on_displayScaleInput_valueChanged(int value);

    void on_showRoadDirectionsInput_toggled(bool checked);

    void on_updateIntervalInput_valueChanged(int value);

    void on_densityInput_valueChanged(int value);

signals:
    void model_updated();
};

#endif // MAINWINDOW_H
