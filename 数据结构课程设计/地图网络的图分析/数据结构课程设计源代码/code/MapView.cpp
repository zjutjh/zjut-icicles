#include "MapView.h"

#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsItem>

// 构造培训每个图形首选项
MapView::MapView(QWidget* parent) : QGraphicsView(parent) {
    setRenderHint(QPainter::Antialiasing, true);
    setRenderHint(QPainter::TextAntialiasing, true);

    setDragMode(QGraphicsView::ScrollHandDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
}

// 鼠标滱轮事件：实现图的缩放功能
void MapView::wheelEvent(QWheelEvent* event) {
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0) {
        event->accept();
        return;
    }

    const qreal zoomInFactor = 1.15;
    const qreal zoomOutFactor = 1.0 / zoomInFactor;

    if (angleDelta.y() > 0) {
        scale(zoomInFactor, zoomInFactor);
    } else {
        scale(zoomOutFactor, zoomOutFactor);
    }
    event->accept();
}

// 鼠标按下事件：检测是否点击了节点
void MapView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        // itemAt 使用视口坐标
        QGraphicsItem* item = itemAt(event->pos());
        if (item) {
            // 我们在 MainWindow::rebuildScene() 里给节点圆点设置了 data(0)=id
            QVariant v = item->data(0);
            if (v.isValid()) {
                bool ok = false;
                int id = v.toInt(&ok);
                if (ok) {
                    emit nodeClicked(id);
                    // 不 return：仍然允许拖拽/选择等默认行为
                }
            }
        }
    }

    QGraphicsView::mousePressEvent(event);
}
