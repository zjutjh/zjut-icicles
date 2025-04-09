// AVLTree.cpp
#ifndef AVLTREE_CPP
#define AVLTREE_CPP

#include "AVLTree.h"
#include <iostream>
// 构造函数
template <typename TKey, typename TValue>
AVLTree<TKey, TValue>::AVLTree() : root(nullptr) {}

// 析构函数
template <typename TKey, typename TValue>
AVLTree<TKey, TValue>::~AVLTree() {
    destroy(root);
}

// 递归销毁节点
template <typename TKey, typename TValue>
void AVLTree<TKey, TValue>::destroy(AVLNode<TKey, TValue>* node) {
    if (node) {
        destroy(node->left);
        destroy(node->right);
        delete node;
    }
}

// 获取节点高度
template <typename TKey, typename TValue>
int AVLTree<TKey, TValue>::getHeight(AVLNode<TKey, TValue>* node) const {
    return node ? node->height : 0;
}

// 获取平衡因子
template <typename TKey, typename TValue>
int AVLTree<TKey, TValue>::getBalanceFactor(AVLNode<TKey, TValue>* node) const {
    if (!node) return 0;
    return getHeight(node->left) - getHeight(node->right);
}

// 左旋
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::rotateLeft(AVLNode<TKey, TValue>* node) {
    AVLNode<TKey, TValue>* newRoot = node->right;
    node->right = newRoot->left;
    newRoot->left = node;

    // 更新高度
    node->height = std::max(getHeight(node->left), getHeight(node->right)) + 1;
    newRoot->height = std::max(getHeight(newRoot->left), getHeight(newRoot->right)) + 1;

    return newRoot;
}

// 右旋
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::rotateRight(AVLNode<TKey, TValue>* node) {
    AVLNode<TKey, TValue>* newRoot = node->left;
    node->left = newRoot->right;
    newRoot->right = node;

    // 更新高度
    node->height = std::max(getHeight(node->left), getHeight(node->right)) + 1;
    newRoot->height = std::max(getHeight(newRoot->left), getHeight(newRoot->right)) + 1;

    return newRoot;
}

// 左右旋
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::rotateLeftRight(AVLNode<TKey, TValue>* node) {
    node->left = rotateLeft(node->left);
    return rotateRight(node);
}

// 右左旋
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::rotateRightLeft(AVLNode<TKey, TValue>* node) {
    node->right = rotateRight(node->right);
    return rotateLeft(node);
}

// 插入节点
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::insert(AVLNode<TKey, TValue>* node, const TKey& key, TValue value) {
    if (!node)
        return new AVLNode<TKey, TValue>(key, value);

    if (key < node->key)
        node->left = insert(node->left, key, value);
    else if (key > node->key)
        node->right = insert(node->right, key, value);
    else
        return node; // 不允许重复键

    // 更新高度
    node->height = 1 + std::max(getHeight(node->left), getHeight(node->right));

    // 获取平衡因子
    int balanceFactor = getBalanceFactor(node);

    // 平衡树
    if (balanceFactor > 1 && key < node->left->key)
        return rotateRight(node); // LL
    if (balanceFactor < -1 && key > node->right->key)
        return rotateLeft(node); // RR
    if (balanceFactor > 1 && key > node->left->key)
        return rotateLeftRight(node); // LR
    if (balanceFactor < -1 && key < node->right->key)
        return rotateRightLeft(node); // RL

    return node;
}

// 公共插入接口
template <typename TKey, typename TValue>
void AVLTree<TKey, TValue>::insert(const TKey& key, TValue value) {
    root = insert(root, key, value);
}

// 查找最小节点
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::findMin(AVLNode<TKey, TValue>* node) const {
    AVLNode<TKey, TValue>* current = node;
    while (current->left != nullptr)
        current = current->left;
    return current;
}

// 删除节点
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::remove(AVLNode<TKey, TValue>* node, const TKey& key) {
    if (!node)
        return node;

    if (key < node->key)
        node->left = remove(node->left, key);
    else if (key > node->key)
        node->right = remove(node->right, key);
    else {
        // 找到节点
        if (!node->left || !node->right) {
            AVLNode<TKey, TValue>* temp = node->left ? node->left : node->right;
            if (!temp) {
                // 无子节点
                temp = node;
                node = nullptr;
            } else {
                // 一个子节点
                *node = *temp;
            }
            delete temp;
        } else {
            // 两个子节点
            AVLNode<TKey, TValue>* temp = findMin(node->right);
            node->key = temp->key;
            node->value = temp->value;
            node->right = remove(node->right, temp->key);
        }
    }

    if (!node)
        return node;

    // 更新高度
    node->height = 1 + std::max(getHeight(node->left), getHeight(node->right));

    // 获取平衡因子
    int balanceFactor = getBalanceFactor(node);

    // 平衡树
    if (balanceFactor > 1 && getBalanceFactor(node->left) >= 0)
        return rotateRight(node); // LL
    if (balanceFactor > 1 && getBalanceFactor(node->left) < 0) {
        node->left = rotateLeft(node->left);
        return rotateRight(node);
    }
    if (balanceFactor < -1 && getBalanceFactor(node->right) <= 0)
        return rotateLeft(node); // RR
    if (balanceFactor < -1 && getBalanceFactor(node->right) > 0) {
        node->right = rotateRight(node->right);
        return rotateLeft(node);
    }

    return node;
}

// 公共删除接口
template <typename TKey, typename TValue>
void AVLTree<TKey, TValue>::remove(const TKey& key) {
    root = remove(root, key);
}

// 搜索节点
template <typename TKey, typename TValue>
TValue AVLTree<TKey, TValue>::search(const TKey& key) const {
    AVLNode<TKey, TValue>* result = search(root, key);
    return result ? result->value : nullptr;
}

// 递归搜索
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::search(AVLNode<TKey, TValue>* node, const TKey& key) const {
    if (!node)
        return nullptr;
    if (key < node->key)
        return search(node->left, key);
    if (key > node->key)
        return search(node->right, key);
    return node;
}

// 获取根节点
template <typename TKey, typename TValue>
AVLNode<TKey, TValue>* AVLTree<TKey, TValue>::getRoot() const {
    return root;
}
template <typename TKey, typename TValue>
void AVLTree<TKey, TValue>::removeRoot(){
    remove(root->key);
}


#endif // AVLTREE_CPP
