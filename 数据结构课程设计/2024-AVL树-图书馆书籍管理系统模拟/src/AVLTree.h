// AVLTree.h
#ifndef AVLTREE_H
#define AVLTREE_H

#include "AVLNode.hpp"

template <typename TKey, typename TValue>
class AVLTree {
public:
    AVLTree();
    ~AVLTree();

    void insert(const TKey& key, TValue value);
    void remove(const TKey& key);
    TValue search(const TKey& key) const;
    AVLNode<TKey, TValue>* getRoot() const;

private:
    AVLNode<TKey, TValue>* root;

    // 内部函数
    AVLNode<TKey, TValue>* insert(AVLNode<TKey, TValue>* node, const TKey& key, TValue value);
    AVLNode<TKey, TValue>* remove(AVLNode<TKey, TValue>* node, const TKey& key);
    AVLNode<TKey, TValue>* search(AVLNode<TKey, TValue>* node, const TKey& key) const;
    AVLNode<TKey, TValue>* findMin(AVLNode<TKey, TValue>* node) const;
    void destroy(AVLNode<TKey, TValue>* node);
    int getHeight(AVLNode<TKey, TValue>* node) const;
    int getBalanceFactor(AVLNode<TKey, TValue>* node) const;
    AVLNode<TKey, TValue>* rotateLeft(AVLNode<TKey, TValue>* node);
    AVLNode<TKey, TValue>* rotateRight(AVLNode<TKey, TValue>* node);
    AVLNode<TKey, TValue>* rotateLeftRight(AVLNode<TKey, TValue>* node);
    AVLNode<TKey, TValue>* rotateRightLeft(AVLNode<TKey, TValue>* node);
    void removeRoot();
};

// 由于这是模板类，所有实现必须在头文件中
#include "AVLTree.cpp"

#endif // AVLTREE_H
