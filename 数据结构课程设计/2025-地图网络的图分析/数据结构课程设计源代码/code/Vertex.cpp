#include "Vertex.h"

// 默认构造函数
Vertex::Vertex() : mId(-1), mName(""), mX(0.0), mY(0.0) {}

// 有参数构造函数
Vertex::Vertex(int id, const std::string& name, double x, double y)
    : mId(id), mName(name), mX(x), mY(y) {}

// 取值函数
int Vertex::id() const { return mId; }
const std::string& Vertex::name() const { return mName; }
double Vertex::x() const { return mX; }
double Vertex::y() const { return mY; }

// 设置顶点名称
void Vertex::setName(const std::string& name) {
    mName = name;
}

// 设置顶点位置
void Vertex::setPosition(double x, double y) {
    mX = x;
    mY = y;
}
