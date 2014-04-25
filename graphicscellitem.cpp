#include "graphicscellitem.h"

/**
* @brief GraphicsCellItem::GraphicsCellItem Initialises a GraphicsCellItem object.
* @param is_empty Specifies whether the cell is empty.
* @param road Road index to represent.
* @param cell Cell index to represent.
* @param x Coordinate on the x-axis.
* @param y Coordinate on the y-axis.
* @param width Width of cell to draw.
* @param height Height of cell to draw.
*/
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

/**
* @brief GraphicsCellItem::has_vehicle Returns true if the represented cell contains a vehicle.
*/
bool GraphicsCellItem::has_vehicle()
{
    return(!this->is_empty);
}

/**
* @brief GraphicsCellItem::setSelected Selects the drawn cell in the GUI, thereby tracking its vehicle if any.
*/
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

/**
 * @brief GraphicsCellItem::get_road Returns the road index of the drawn cell.
 */
unsigned int GraphicsCellItem::get_road()
{
    return this->road;
}

/**
 * @brief GraphicsCellItem::get_cell Returns the cell index of the drawn cell.
 */
unsigned int GraphicsCellItem::get_cell()
{
    return this->cell;
}

/**
* @brief GraphicsCellItem::vehicle_brush Returns a coloured QBrush for vehicle cells.
*/
QBrush GraphicsCellItem::vehicle_brush()
{
    return QBrush(Qt::green);
}

/**
* @brief GraphicsCellItem::focused_vehicle_brush Returns a coloured QBrush for tracked vehicle cells.
*/
QBrush GraphicsCellItem::focused_vehicle_brush()
{
    return QBrush(Qt::red);
}

/**
* @brief GraphicsCellItem::empty_cell_brush Returns an uncoloured QBrush for empty cells.
*/
QBrush GraphicsCellItem::empty_cell_brush()
{
    return QBrush(Qt::white);
}

/**
* @brief qgraphicsitem_cast Allows casting of QGraphicsItem to GraphicsCellItem.
*/
GraphicsCellItem* qgraphicsitem_cast(QGraphicsItem *item)
{
    return (GraphicsCellItem*)item;
}
