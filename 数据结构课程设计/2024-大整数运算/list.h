#ifndef LIST_H
#define LIST_H

#ifdef _WIN32
    using size_t = unsigned long long;
#elif __linux__
    using size_t = unsigned long;
#else
    #error "Unsupported platform"
#endif

template <typename T> class list {
private:
  struct Node {
    T data;
    Node *next;
    Node *prev;
    Node(const T &value) : data(value), next(nullptr), prev(nullptr) {}
  };

public:
  Node *head;
  Node *tail;
  size_t siz;

  list() : head(nullptr), tail(nullptr), siz(0) {}
  ~list() { clear(); }
  list(const list<T> &other) : head(nullptr), tail(nullptr), siz(0) {
    for (const T &item : other) {
      push_back(item);
    }
  }
  list &operator=(const list<T> &other) {
    if (this != &other) {
      clear();
      for (const T &item : other) {
        push_back(item);
      }
    }
    return *this;
  }
  size_t size() const { return siz; }
  bool empty() const { return siz == 0; }

  void clear() {
    Node *cur = head;
    while (cur) {
      Node *next = cur->next;
      delete cur;
      cur = next;
    }
    head = tail = nullptr;
    siz = 0;
  }

  void push_back(const T &value) {
    Node *newNode = new Node(value);
    if (!tail) {
      head = tail = newNode;
    } else {
      tail->next = newNode;
      newNode->prev = tail;
      tail = newNode;
    }
    siz++;
  }
  void push_front(const T &value) {
    Node *newNode = new Node(value);
    if (!head) {
      head = tail = newNode;
    } else {
      newNode->next = head;
      head->prev = newNode;
      head = newNode;
    }
    siz++;
  }
  void pop_back() {
    if (!tail) {
      throw "pop_back on empty list";
    }
    Node *oldTail = tail;
    tail = tail->prev;
    if (tail) {
      tail->next = nullptr;
    } else {
      head = nullptr;
    }
    delete oldTail;
    siz--;
  }
  void pop_front() {
    if (!head) {
      throw "pop_front on empty list";
    }
    Node *oldHead = head;
    head = head->next;
    if (head) {
      head->prev = nullptr;
    } else {
      tail = nullptr;
    }
    delete oldHead;
    siz--;
  }

  T &back() const {
    if (!tail) {
      throw "back on empty list";
    }
    return tail->data;
  }
  T &front() const {
    if (!head) {
      throw "front on empty list";
    }
    return head->data;
  }

  class Iterator {
  private:
    Node *cur;

  public:
    Iterator(Node *node) : cur(node) {};

    T &operator*() const {
      if (!cur) {
        throw "Dereferencing a null iterator";
      }
      return cur->data;
    }
    Iterator &operator++() {
      if (cur) {
        cur = cur->next;
      }
      return *this;
    }
    Iterator operator++(int) {
      Iterator temp = *this;
      ++(*this);
      return temp;
    }
    Iterator &operator--() {
      if (cur) {
        cur = cur->prev;
      }
      return *this;
    }
    Iterator operator--(int) {
      Iterator temp = *this;
      --(*this);
      return temp;
    }
    bool operator==(const Iterator &other) const {
      return cur == other.cur;
    }
    bool operator!=(const Iterator &other) const { return !(*this == other); }
  };

  Iterator begin() const { return Iterator(head); }
  Iterator end() const { return Iterator(nullptr); }
  Iterator rbegin() const { return Iterator(tail); }
};
#endif