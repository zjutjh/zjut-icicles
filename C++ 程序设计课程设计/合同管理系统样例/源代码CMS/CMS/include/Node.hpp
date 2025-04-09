#ifndef NODE_HPP
#define NODE_HPP

#include "ReadFile.hpp"

class Node {
public:
    Contract contract;
    Node* next;

    Node(const Contract& contract) : contract(contract), next(nullptr) {}
};

#endif // NODE_HPP
