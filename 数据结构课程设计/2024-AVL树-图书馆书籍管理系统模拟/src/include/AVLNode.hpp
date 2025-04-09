// AVLNode.hpp
#ifndef AVLNODE_HPP
#define AVLNODE_HPP

template <typename TKey, typename TValue>
struct AVLNode {
    TKey key;
    TValue value;
    AVLNode* left;
    AVLNode* right;
    int height;
    AVLNode(const TKey& key, const TValue& value)
        : key(key), value(value), left(nullptr), right(nullptr), height(1) {}
};
#endif // AVLNODE_HPP
