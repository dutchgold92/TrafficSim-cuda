#include "graphicscellitem.h"

GraphicsCellItem::GraphicsCellItem(bool is_empty, unsigned int road, unsigned int cell, qreal x, qreal y, qreal width, qreal height) :
    QGraphicsRectItem(x, y, width, height)
{
    this->is_empty = is_empty;
    this->road = road;
    this->cell = cell;

    if(this->has_vehicle())
    {
        this->setBrush(this->vehicle_brush());
        this->setFlag(QGraphicsItem::ItemIsSelectable, true);
    }
    else
    {
        this->setBrush(this->empty_cell_brush());
        this->setFlag(QGraphicsItem::ItemIsSelectable, false);
    }
}

bool GraphicsCellItem::has_vehicle()
{
    return(!this->is_empty);
}

void GraphicsCellItem::setSelected(bool selected)
{
    if(this->has_vehicle())
    {
        if(selected)
            this->setBrush(this->focused_vehicle_brush());
        else
            this->setBrush(this->vehicle_brush());
    }
}

unsigned int GraphicsCellItem::get_road()
{
    return this->road;
}

unsigned int GraphicsCellItem::get_cell()
{
    return this->cell;
}

QBrush GraphicsCellItem::vehicle_brush()
{
    return QBrush(Qt::green);
}

QBrush GraphicsCellItem::focused_vehicle_brush()
{
    return QBrush(Qt::red);
}

QBrush GraphicsCellItem::empty_cell_brush()
{
    return QBrush(Qt::white);
}

GraphicsCellItem* qgraphicsitem_cast(QGraphicsItem *item)
{
    return (GraphicsCellItem*)item;
}
