//
// Created by Dylan on 2021/12/15.
//

#ifndef AVL_NODE_H
#define AVL_NODE_H
#include <string>
using namespace std;
struct userInfo{  //用户数据结构体
    string username;
    string password;
};

class Node {
private:
    int bf = 0;  //结点平衡因子
    Node *lChild = nullptr, *rChild = nullptr;  //左右孩子指针
    userInfo user;  //用户信息

public:
    Node(string usn, string pwd);  //初始化
    userInfo& getUser();  //获取用户信息
    Node* getLChild();  //获取左孩子
    Node* getRChild();  //获取右孩子
    int getBF();  //获取平衡因子
    void setBf(int bf);  //设置平衡因子
    void setLChild(Node* newNode);  //添加左孩子
    void setRChild(Node* newNode);  //添加右孩子
    ~Node();  //释放结点
};


#endif //AVL_NODE_H
