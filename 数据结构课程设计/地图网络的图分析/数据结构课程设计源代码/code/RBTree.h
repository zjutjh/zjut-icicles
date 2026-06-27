#pragma once
#include <cstddef>
#include <functional>
#include <utility>

// 红黑树模板类：一个自平衡的二叉搜索树实现
template <typename Key, typename Value, typename Compare = std::less<Key>>
class RBTree {
public:
    RBTree() {
        initNil();
        mRoot = mNil;
        mSize = 0;
    }

    RBTree(const RBTree& other) = delete;
    RBTree& operator=(const RBTree& other) = delete;

    RBTree(RBTree&& other) noexcept {
        // 移动：直接接管指针（简单处理）
        mNil = other.mNil;
        mRoot = other.mRoot;
        mSize = other.mSize;
        mComp = std::move(other.mComp);

        other.mNil = nullptr;
        other.mRoot = nullptr;
        other.mSize = 0;
    }

    RBTree& operator=(RBTree&& other) noexcept {
        if (this == &other) return *this;
        destroyAll();

        mNil = other.mNil;
        mRoot = other.mRoot;
        mSize = other.mSize;
        mComp = std::move(other.mComp);

        other.mNil = nullptr;
        other.mRoot = nullptr;
        other.mSize = 0;
        return *this;
    }

    ~RBTree() {
        destroyAll();
    }

    std::size_t size() const { return mSize; }
    bool empty() const { return mSize == 0; }

    void clear() {
        if (!mNil) return; // 兼容 move 后对象
        clearSubtree(mRoot);
        mRoot = mNil;
        mSize = 0;
    }

    // 插入：若key不存在则插入，存在则不改value，返回是否截插
    bool insert(const Key& key, const Value& value) {
        return insertImpl(key, value, false);
    }

    // 插入或赋值：若key存在则覆盖value，若不存在则插入
    void insertOrAssign(const Key& key, const Value& value) {
        insertImpl(key, value, true);
    }

    // 检查是否包含key
    bool contains(const Key& key) const {
        return findNode(key) != mNil;
    }

    // 查找：找到返回true，并将value写入outValue；找不到返回false
    bool tryGet(const Key& key, Value& outValue) const {
        Node* node = findNode(key);
        if (node == mNil) return false;
        outValue = node->value;
        return true;
    }

    // 返回指针，找不到返回 nullptr（便于你后面禁用集合做标记）
    const Value* find(const Key& key) const {
        Node* node = findNode(key);
        if (node == mNil) return nullptr;
        return &node->value;
    }

    Value* find(const Key& key) {
        Node* node = findNode(key);
        if (node == mNil) return nullptr;
        return &node->value;
    }

    // 删除：存在则删除并返回 true，否则 false
    bool erase(const Key& key) {
        Node* z = findNode(key);
        if (z == mNil) return false;

        Node* y = z;
        Color yOriginalColor = y->color;
        Node* x = mNil;

        if (z->left == mNil) {
            x = z->right;
            transplant(z, z->right);
        } else if (z->right == mNil) {
            x = z->left;
            transplant(z, z->left);
        } else {
            y = minimum(z->right);
            yOriginalColor = y->color;
            x = y->right;
            if (y->parent == z) {
                x->parent = y;
            } else {
                transplant(y, y->right);
                y->right = z->right;
                y->right->parent = y;
            }

            transplant(z, y);
            y->left = z->left;
            y->left->parent = y;
            y->color = z->color;
        }

        delete z;
        mSize--;

        if (yOriginalColor == Color::Black) {
            deleteFixup(x);
        }
        return true;
    }

    // 中序遍历（调试/导出用）
    template <typename Func>
    void inorder(Func func) const {
        inorderImpl(mRoot, func);
    }

private:
    enum class Color { Red, Black };

    struct Node {
        Key key;
        Value value;
        Color color = Color::Red;
        Node* parent = nullptr;
        Node* left = nullptr;
        Node* right = nullptr;

        Node() = default;
        Node(const Key& k, const Value& v) : key(k), value(v) {}
    };

    Node* mNil = nullptr;   // 哨兵 NIL（黑色）
    Node* mRoot = nullptr;
    std::size_t mSize = 0;
    Compare mComp = Compare();

private:
    void initNil() {
        mNil = new Node();
        mNil->color = Color::Black;
        mNil->parent = mNil;
        mNil->left = mNil;
        mNil->right = mNil;
    }

    void destroyAll() {
        if (!mNil) return;
        clear();
        delete mNil;
        mNil = nullptr;
        mRoot = nullptr;
    }

    bool isEqualKey(const Key& a, const Key& b) const {
        return !mComp(a, b) && !mComp(b, a);
    }

    Node* findNode(const Key& key) const {
        if (!mNil) return nullptr; // move 后保护
        Node* cur = mRoot;
        while (cur != mNil) {
            if (isEqualKey(key, cur->key)) return cur;
            if (mComp(key, cur->key)) cur = cur->left;
            else cur = cur->right;
        }
        return mNil;
    }

    bool insertImpl(const Key& key, const Value& value, bool assignIfExists) {
        Node* y = mNil;
        Node* x = mRoot;

        while (x != mNil) {
            y = x;
            if (isEqualKey(key, x->key)) {
                if (assignIfExists) {
                    x->value = value;
                }
                return false; // 已存在
            }
            if (mComp(key, x->key)) x = x->left;
            else x = x->right;
        }

        Node* z = new Node(key, value);
        z->left = mNil;
        z->right = mNil;
        z->parent = y;
        z->color = Color::Red;

        if (y == mNil) {
            mRoot = z;
        } else if (mComp(key, y->key)) {
            y->left = z;
        } else {
            y->right = z;
        }

        mSize++;
        insertFixup(z);
        return true;
    }

    void rotateLeft(Node* x) {
        Node* y = x->right;
        x->right = y->left;
        if (y->left != mNil) {
            y->left->parent = x;
        }
        y->parent = x->parent;

        if (x->parent == mNil) {
            mRoot = y;
        } else if (x == x->parent->left) {
            x->parent->left = y;
        } else {
            x->parent->right = y;
        }

        y->left = x;
        x->parent = y;
    }

    void rotateRight(Node* y) {
        Node* x = y->left;
        y->left = x->right;
        if (x->right != mNil) {
            x->right->parent = y;
        }
        x->parent = y->parent;

        if (y->parent == mNil) {
            mRoot = x;
        } else if (y == y->parent->right) {
            y->parent->right = x;
        } else {
            y->parent->left = x;
        }

        x->right = y;
        y->parent = x;
    }

    void insertFixup(Node* z) {
        while (z->parent->color == Color::Red) {
            if (z->parent == z->parent->parent->left) {
                Node* y = z->parent->parent->right; // uncle
                if (y->color == Color::Red) {
                    z->parent->color = Color::Black;
                    y->color = Color::Black;
                    z->parent->parent->color = Color::Red;
                    z = z->parent->parent;
                } else {
                    if (z == z->parent->right) {
                        z = z->parent;
                        rotateLeft(z);
                    }
                    z->parent->color = Color::Black;
                    z->parent->parent->color = Color::Red;
                    rotateRight(z->parent->parent);
                }
            } else {
                Node* y = z->parent->parent->left; // uncle
                if (y->color == Color::Red) {
                    z->parent->color = Color::Black;
                    y->color = Color::Black;
                    z->parent->parent->color = Color::Red;
                    z = z->parent->parent;
                } else {
                    if (z == z->parent->left) {
                        z = z->parent;
                        rotateRight(z);
                    }
                    z->parent->color = Color::Black;
                    z->parent->parent->color = Color::Red;
                    rotateLeft(z->parent->parent);
                }
            }
        }
        mRoot->color = Color::Black;
        mRoot->parent = mNil;
    }

    void transplant(Node* u, Node* v) {
        if (u->parent == mNil) {
            mRoot = v;
        } else if (u == u->parent->left) {
            u->parent->left = v;
        } else {
            u->parent->right = v;
        }
        v->parent = u->parent;
    }

    Node* minimum(Node* x) const {
        while (x->left != mNil) x = x->left;
        return x;
    }

    void deleteFixup(Node* x) {
        while (x != mRoot && x->color == Color::Black) {
            if (x == x->parent->left) {
                Node* w = x->parent->right;
                if (w->color == Color::Red) {
                    w->color = Color::Black;
                    x->parent->color = Color::Red;
                    rotateLeft(x->parent);
                    w = x->parent->right;
                }

                if (w->left->color == Color::Black && w->right->color == Color::Black) {
                    w->color = Color::Red;
                    x = x->parent;
                } else {
                    if (w->right->color == Color::Black) {
                        w->left->color = Color::Black;
                        w->color = Color::Red;
                        rotateRight(w);
                        w = x->parent->right;
                    }
                    w->color = x->parent->color;
                    x->parent->color = Color::Black;
                    w->right->color = Color::Black;
                    rotateLeft(x->parent);
                    x = mRoot;
                }
            } else {
                Node* w = x->parent->left;
                if (w->color == Color::Red) {
                    w->color = Color::Black;
                    x->parent->color = Color::Red;
                    rotateRight(x->parent);
                    w = x->parent->left;
                }

                if (w->right->color == Color::Black && w->left->color == Color::Black) {
                    w->color = Color::Red;
                    x = x->parent;
                } else {
                    if (w->left->color == Color::Black) {
                        w->right->color = Color::Black;
                        w->color = Color::Red;
                        rotateLeft(w);
                        w = x->parent->left;
                    }
                    w->color = x->parent->color;
                    x->parent->color = Color::Black;
                    w->left->color = Color::Black;
                    rotateRight(x->parent);
                    x = mRoot;
                }
            }
        }
        x->color = Color::Black;
        mRoot->parent = mNil;
    }

    void clearSubtree(Node* node) {
        if (node == mNil) return;
        clearSubtree(node->left);
        clearSubtree(node->right);
        delete node;
    }

    template <typename Func>
    void inorderImpl(Node* node, Func func) const {
        if (node == mNil) return;
        inorderImpl(node->left, func);
        func(node->key, node->value);
        inorderImpl(node->right, func);
    }
};
