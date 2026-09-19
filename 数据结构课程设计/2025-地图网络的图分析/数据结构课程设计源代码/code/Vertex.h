#pragma once
#include <string>

// 顶点类：表示图中的一个顶点（节点）
class Vertex {
public:
    Vertex();
    Vertex(int id, const std::string& name, double x = 0.0, double y = 0.0);

    // 获取方法
    int id() const;
    const std::string& name() const;
    double x() const;
    double y() const;

    // 设置方法
    void setName(const std::string& name);
    void setPosition(double x, double y);

private:
    int mId;
    std::string mName;
    double mX;
    double mY;
};
