//
// Created by Dylan on 2021/12/15.
//

#include "Node.h"

Node::Node(string usn, string pwd) {  //初始化用户信息
    user.username = usn;
    user.password = pwd;
}

userInfo& Node::getUser(){
    return user;
}

Node::~Node() {

}

Node* Node::getLChild() {
    return lChild;
}

Node* Node::getRChild() {
    return rChild;
}

void Node::setLChild(Node *newNode) {
    lChild = newNode;
}

void Node::setRChild(Node *newNode) {
    rChild = newNode;
}

void Node::setBf(int bf) {
    this->bf = bf;
}

int Node::getBF() {
    return bf;
}
