#pragma once
#include <stdexcept>
#include <vector>
template <class T>
class LinkedList {
public:
    struct Node {
        T data{};
        Node *prev = nullptr;
        Node *next = nullptr;
        Node(const T &value) : data(value) {}
    };

    class Iterator {
    public:
        Iterator(Node *position = nullptr) : node(position) {}
        Iterator operator+(int offset) const {
            Node *current = node;
            if (offset >= 0) {
                while (offset-- > 0 && current != nullptr) {
                    current = current->next;
                }
            } else {
                while (offset++ < 0 && current != nullptr) {
                    current = current->prev;
                }
            }
            return Iterator(current);
        }
        operator Node *() const { return node; }

    private:
        Node *node = nullptr;
    };

    LinkedList() {}
    LinkedList(const std::vector<T> &vec);
    LinkedList(const LinkedList &other);

    LinkedList<T> &operator=(LinkedList other);
    ~LinkedList() { clear(); }

    void push_back(const T &value);
    void clear();
    bool empty() const { return nodeCount == 0; }
    int size() const { return nodeCount; }
    void insert(Iterator position, const T &value);
    void erase(Iterator position);

    Iterator begin() const { return Iterator(head); }
    Iterator end() const { return Iterator(nullptr); }

    T &operator[](int index);
    operator std::vector<T>() const;

private:
    Node *head = nullptr;
    Node *tail = nullptr;
    int nodeCount = 0;
};

template <class T>
inline LinkedList<T>::LinkedList(const std::vector<T> &vec) {
    for (const T &value : vec) {
        push_back(value);
    }
}

template <class T>
inline LinkedList<T>::LinkedList(const LinkedList &other) {
    Node *current = other.head;
    while (current != nullptr) {
        push_back(current->data);
        current = current->next;
    }
}

template <class T>
inline LinkedList<T> &LinkedList<T>::operator=(LinkedList other) {
    // copy-and-swap
    std::swap(this->head, other.head);
    std::swap(this->tail, other.tail);
    std::swap(this->nodeCount, other.nodeCount);
    return *this;
}

template <class T>
inline void LinkedList<T>::push_back(const T &value) {
    Node *newNode = new Node(value);
    if (!head) {
        head = tail = newNode;
    } else {
        tail->next = newNode;
        newNode->prev = tail;
        tail = newNode;
    }
    ++nodeCount;
}

template <class T>
inline void LinkedList<T>::clear() {
    Node *current = head;
    while (current) {
        Node *nextNode = current->next;
        delete current;
        current = nextNode;
    }
    head = tail = nullptr;
    nodeCount = 0;
}

// Inserts given value into LinkedList BEFORE specified iterator.
// (to work with std::vector)
template <class T>
inline void LinkedList<T>::insert(Iterator position, const T &value) {
    Node *positionNode = position;
    if (positionNode == nullptr) {
        push_back(value);
        return;
    }
    Node *newNode = new Node(value);
    newNode->next = positionNode;
    newNode->prev = positionNode->prev;
    positionNode->prev = newNode;
    if (newNode->prev != nullptr) {
        newNode->prev->next = newNode;
    } else {
        head = newNode;
    }
    ++nodeCount;
}

template <class T>
inline void LinkedList<T>::erase(Iterator position) {
    Node *positionNode = position;
    if (positionNode == nullptr) {
        return;
    }
    if (positionNode->prev) {
        positionNode->prev->next = positionNode->next;
    } else {
        head = positionNode->next;
    }
    if (positionNode->next) {
        positionNode->next->prev = positionNode->prev;
    } else {
        tail = positionNode->prev;
    }
    delete positionNode;
    --nodeCount;
}

template <class T>
inline T &LinkedList<T>::operator[](int index) {
    if (index < 0 || index >= nodeCount) {
        throw std::out_of_range("index out of range");
    }
    Node *current = head;
    for (int i = 0; i < index; ++i) {
        current = current->next;
    }
    return current->data;
}

template <class T>
inline LinkedList<T>::operator std::vector<T>() const {
    std::vector<T> vec;
    vec.reserve(nodeCount);
    Node *current = head;
    while (current) {
        vec.push_back(current->data);
        current = current->next;
    }
    return vec;
}
