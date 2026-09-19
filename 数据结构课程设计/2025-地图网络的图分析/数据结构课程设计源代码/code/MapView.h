#pragma once
#include <QGraphicsView>

class QWheelEvent;
class QMouseEvent;

// 地图视图：自定义QGraphicsView，支持缩放和节点点击
class MapView : public QGraphicsView {
    Q_OBJECT
public:
    explicit MapView(QWidget* parent = nullptr);

signals:
    // 当节点被点击时发出此信号
    void nodeClicked(int nodeId);

protected:
    // 鼠标滚轮事件：实现缩放功能
    void wheelEvent(QWheelEvent* event) override;
    // 鼠标按下事件：检测节点点击
    void mousePressEvent(QMouseEvent* event) override;
};
