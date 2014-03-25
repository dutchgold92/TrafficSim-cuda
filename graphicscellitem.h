#ifndef GRAPHICSCELLITEM_H
#define GRAPHICSCELLITEM_H

#include <QGraphicsRectItem>
#include <QBrush>
#include <QPen>
#include <model.h>

class GraphicsCellItem : public QObject, public QGraphicsRectItem
{
    Q_OBJECT
private:
    QBrush vehicle_brush();
    QBrush focused_vehicle_brush();
    QBrush empty_cell_brush();
    bool is_empty;
    unsigned int road;
    unsigned int cell;
public:
    explicit GraphicsCellItem(bool is_empty, unsigned int road, unsigned int cell, qreal x, qreal y, qreal width, qreal height);
    enum {Type = UserType + 1};
    bool has_vehicle();
    void setSelected(bool selected);
    unsigned int get_road();
    unsigned int get_cell();
    int type() const
    {
        return Type;
    }
};

#endif // GRAPHICSCELLITEM_H
