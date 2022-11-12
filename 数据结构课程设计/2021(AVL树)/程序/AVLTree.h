//
// Created by Dylan on 2021/12/15.
//

#ifndef AVL_AVLTREE_H
#define AVL_AVLTREE_H
#include <iostream>
#include <fstream>
#include <algorithm>
#include "Node.h"

class AVLTree {
private:
    Node *root = nullptr;  //指向树根
    int depth = 0;  //二叉树的深度
    bool sign = false;  //记录操作是否成功

    // 一个数组，打印用，数组长度不低于二叉树的高度，设为一百
    // 标记当前的节点是父节点的左孩子还是右孩子
    int vec_left[100] = {0};

    void printAVL(Node *node, int sign = 0);  //打印二叉树
    void deleteNode(Node *node, string usn, Node *p = nullptr);  //删除结点
    void add(Node *T, Node *newNode, Node *parent= nullptr);  //添加结点
    void updatePassword(string usn, string pwd, Node *T);  //更新密码
    void login(string usn, string pwd, Node *T);  //登录
    void save(ofstream &outFile, Node *T);
public:
    AVLTree(ifstream &inFile);  //使用文件对象初始化二叉树
    ~AVLTree();  //二叉树的释放

    void save(ofstream &outFile);  //将用户信息存入文件
    void print();  //封装后的打印函数

    bool getSign();  //判断操作是否成功
    void reSign();  //重置标志

    int getHeight(Node *T);  //获取树的高度
    void deleteNode(string usn);  //封装后的删除结点函数
    void add(string usn, string pwd);  //封装后的添加结点函数
    void updatePassword(string usn, string pwd);  //封装后的更新密码函数
    void login(string usn, string pwd);  //封装后的登录函数

    void RRotate(Node *node, Node *parent);  //右旋
    void LRotate(Node *node, Node *parent);  //左旋
};


#endif //AVL_AVLTREE_H
