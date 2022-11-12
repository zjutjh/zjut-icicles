//
// Created by Dylan on 2021/12/15.
//

#include "AVLTree.h"
AVLTree::AVLTree(ifstream &inFile) {
    string tempstr;  //读取字符串
    int searchBound;  //记录字符串分割标志"@"的位置
    Node *temp;  //新结点指针

    //用户初始化
    while(inFile.peek() != EOF){  //检测文件是否到底
        //字符处理
        inFile>>tempstr;
        searchBound = tempstr.find('@');

        //新建结点并初始化
        temp = new Node(tempstr.substr(0,searchBound),  //获取用户名
                        tempstr.substr(searchBound+1,tempstr.length()-searchBound-1)  //获取密码
                        );

        //在树中插入新结点
        add(root, temp);
    }
    sign = false;
}

AVLTree::~AVLTree() {

}

void AVLTree::printAVL(Node *node, int sign) {
    if(sign> 0){  //当树的高度大于0时
        for(int i = 0; i < sign - 1; i++){
            printf(vec_left[i] ? "│   " : "    ");
        }
        printf(vec_left[sign-1] ? "├── " : "└── ");
    }

    if(! node){  //当结点不存在的时候
        printf("(null)\n");
        return;
    }

    //打印当前结点数据
    printf("%s\n", node->getUser().username.c_str());

    //当前结点为叶子结点时返回
    if(!node->getLChild() && !node->getRChild()){
        return;
    }

    vec_left[sign] = 1; //进入当前结点的右孩子
    printAVL(node->getRChild(), sign + 1);
    vec_left[sign] = 0; //进入当前结点的左孩子
    printAVL(node->getLChild(), sign + 1);

}

void AVLTree::print() {  //封装后的打印函数
    printAVL(root);
}




void AVLTree::add(Node *T, Node *newNode, Node *parent) {  //第一个参数是当前结点地址，作递归用，第二个参数是新结点地址
    //AVL树是空树时，头节点指向加入的结点
    if(root == nullptr) {
        root = newNode;
        //获取树的深度
        depth = getHeight(root);
        sign = true;
        return;
    }


    //AVL不为空时
    if(T->getUser().username>newNode->getUser().username){  //若当前结点值比新结点值大，则转向左孩子

        if(T->getLChild() == nullptr){
            T->setLChild(newNode);  //左孩子为空，直接添加为左孩子
            sign = true;
            //获取树的深度
            depth = getHeight(root);
        } else add(T->getLChild(), newNode, T);  //左孩子非空，转向左孩子

    }else if(T->getUser().username<newNode->getUser().username){  //若当前结点值比新结点值小，则转向右孩子

        if(T->getRChild() == nullptr){
            T->setRChild(newNode);  //右孩子为空，直接添加为右孩子
            sign = true;
            //获取树的深度
            depth = getHeight(root);
        } else add(T->getRChild(), newNode, T);  //右孩子非空，转向右孩子

    }else{  //若当前结点值和新结点值一样大，则返回错误信息
        cout<<"用户\""<<newNode->getUser().username<<"\"已存在！"<<endl;
        delete newNode;  //释放空间
        return;
    }

    //平衡操作
    if(T->getBF() > 1){  //找到最小不平衡子树
        RRotate(T, parent);  //右旋
    }else if(T->getBF() < -1){
        LRotate(T, parent);  //左旋
    }
}



/*
 * 由于获取树的深度时需要得到每个结点左右子树的深度
 * 所以可以顺便计算出当前结点的平衡因子
 */
int AVLTree::getHeight(Node *T) {  //使用递归获取树的深度
    if(T == nullptr) return 0;

    int left, right;  //记录左右子树深度

    left = getHeight(T->getLChild());  //当前结点左子树深度
    right = getHeight(T->getRChild());  //当前结点右子树深度
    T->setBf(left-right);  //计算出当前结点的平衡因子

    return  max(left, right)+1;  //取左右子树中最深的一个作为返回值
}

void AVLTree::deleteNode(Node *node, string usn, Node *p) {
    if(node == nullptr){
        //cout<<"empty node!"<<endl;
        return;
    }

    if(node->getUser().username == usn){  //若当前结点即为所寻结点

        if(node->getRChild() && node->getLChild()) {//如果有两个孩子结点

            Node* temPreNode = node;  //指向中序后继结点的双亲结点
            Node* temNode = node->getRChild();  //指向中序后继结点
            while(temNode->getLChild() != nullptr){  //遍历到右子树最左下结点
                temPreNode = temNode;
                temNode = temNode->getLChild();
            }

            if(node->getRChild() != temNode){  //当中序后继结点不是目标结点的右孩子时
                if(temNode->getRChild() != nullptr){  //若中序后继节点右孩子不为空
                    temPreNode->setLChild(temNode->getRChild());  //中序后继结点的右孩子变为前一个结点的左孩子
                }
                temNode->setRChild(node->getRChild());  //被删除结点的右孩子作为中序后继节点的右孩子
                temPreNode->setLChild(nullptr);  //中序后继结点的双亲结点左孩子置空
            }
            temNode->setLChild(node->getLChild());  //被删除结点的左孩子作为中序后继节点的左孩子

            if(p != nullptr){
                if(p->getRChild() == node) {  //是结点的右孩子时
                    p->setRChild(temNode);  //中序后继节点作为双亲结点的右孩子
                }else if(p->getLChild() == node){  //是结点的左孩子时
                    p->setLChild(temNode);  //中序后继节点作为双亲结点的左孩子
                }
            }else{
                root = temNode;
            }
            delete node;  //删除本结点

        } else if(node->getRChild() && !node->getLChild()){  //如果当前结点只有右孩子

            if(p == nullptr) {  //考虑目标结点为根节点的情形
                root = node->getRChild();  //设置根节点
                delete node;  //删除结点
            }else if(p->getLChild() == node){  //若为父节点的左孩子
                p->setLChild(node->getRChild());
                delete node;  //删除结点
            }else if(p->getRChild() == node){  //若为父节点的右孩子
                p->setRChild(node->getRChild());
                delete node;  //删除结点
            }

        } else if(!node->getRChild() && node->getLChild()){  //如果当前结点只有左孩子

            if(p == nullptr) {  //考虑目标结点为根节点的情形
                root = node->getLChild();  //设置根节点
                delete node;  //删除结点
            }else if(p->getLChild() == node){  //若为父节点的左孩子
                p->setLChild(node->getLChild());
                delete node;  //删除结点
            }else if(p->getRChild() == node){  //若为父节点的右孩子
                p->setRChild(node->getLChild());
                delete node;  //删除结点
            }

        } else{  //若为叶子结点，则直接删除

            if(p == nullptr){  //如果目标为孤立的根节点，则直接删除
                delete node;
                root = nullptr;
            }else if(p->getLChild() == node){  //若为父节点的左孩子
                delete node;  //删除结点
                p->setLChild(nullptr);  //父节点左孩子指针置空
            }else if(p->getRChild() == node){  //若为父节点的右孩子
                delete node;  //删除结点
                p->setRChild(nullptr);  //父节点右孩子指针置空
            }

        }
        node = nullptr;
        //重新计算高度
        depth = getHeight(root);
        sign = true;
        return;
    }

    deleteNode(node->getLChild(), usn, node);

    //原结点位置平衡操作
    while(node->getLChild() != nullptr && node->getLChild()->getBF() > 1) {
        RRotate(node, p);  //右旋
    }

    //在左支删除时的上层平衡操作
    if(node->getBF() > 1){  //找到最小不平衡子树
        RRotate(node, p);  //右旋
    }else if(node->getBF() < -1){
        LRotate(node, p);  //左旋
    }

    deleteNode(node->getRChild(), usn, node);

    //原结点位置平衡操作
    while(node->getRChild() != nullptr && node->getRChild()->getBF() > 1) {
        RRotate(node, p);  //右旋
    }

    //在右支删除时的上层平衡操作
    if(node->getBF() > 1){  //找到最小不平衡子树
        RRotate(node, p);  //右旋
    }else if(node->getBF() < -1){
        LRotate(node, p);  //左旋
    }

}

void AVLTree::deleteNode(string usn) {
    deleteNode(root, usn);
}

void AVLTree::RRotate(Node *node, Node *parent) {
    if(node == root){  //若最小不平衡子树根结点为根节点
        if(node->getLChild()->getBF() < 0){  //如果符号不同，则子结点先进行左旋
            LRotate(node->getLChild(), node);
        }
        Node *temNode = root;
        root = root->getLChild();

        if(root->getRChild() == nullptr){  //如果孩子结点没有右孩子
            root->setRChild(temNode);
            temNode->setLChild(nullptr);
        }else{  //如果孩子结点有右孩子，则将孩子结点的右孩子移到双亲结点的左下角
            temNode->setLChild(root->getRChild());
            root->setRChild(temNode);

        }

    } else{  //若不为根节点
        if(node->getLChild()->getBF() < 0){  //如果符号不同，则子结点先进行左旋
            if(parent->getLChild() == node){  //左边的右旋，双亲结点走左边
                LRotate(node->getLChild(), parent->getLChild());
            }else{  //右边的右旋，双亲结点走右边
                LRotate(node->getLChild(), parent->getRChild());
            }
        }

        //旋转操作
        Node *temNode = node;
        Node *childNode = node->getLChild();

        if(parent->getLChild() == node){  //左边的右旋
            parent->setLChild(childNode);
        }else{  //右边的右旋
            parent->setRChild(childNode);
        }

        if(childNode->getRChild() == nullptr){  //如果孩子结点没有右孩子
            childNode->setRChild(temNode);
            temNode->setLChild(nullptr);
        }else{  //如果孩子结点有右孩子结点，则将孩子结点的右孩子移到目标结点的左下角，进行右旋操作
            temNode->setLChild(childNode->getRChild());
            childNode->setRChild(temNode);
        }

    }
    depth = getHeight(root);  //重新计算树的深度和bf
}

void AVLTree::LRotate(Node *node, Node *parent) {
    if(node == root){  //若最小不平衡子树根结点为根节点
        if (node->getRChild()->getBF() > 0) {  //如果符号不同，则子结点先进行右旋
            RRotate(node->getRChild(), node);
        }
        Node *temNode = root;
        root = root->getRChild();

        if(root->getLChild() == nullptr){  //如果孩子结点没有左孩子
            root->setLChild(temNode);
            temNode->setRChild(nullptr);
        }else{  //如果孩子结点有两个孩子结点，则将孩子结点的左孩子移到双亲结点的右下角
            temNode->setRChild(root->getLChild());
            root->setLChild(temNode);
        }

    } else {  //若不为根节点
        if (node->getRChild()->getBF() > 0) {  //如果符号不同，则子结点先进行右旋
            if(parent->getLChild() == node){  //左边的右旋，双亲结点走左边
                RRotate(node->getRChild(), parent->getLChild());
            }else{  //右边的左旋，双亲结点走右边
                RRotate(node->getRChild(), parent->getRChild());
            }
        }
        //旋转操作
        Node *temNode = node;
        Node *childNode = node->getRChild();

        if(parent->getLChild() == node){  //左边的左旋
            parent->setLChild(childNode);
        }else{  //右边左旋
            parent->setRChild(childNode);
        }

        if(childNode->getLChild() == nullptr){  //如果孩子结点没有左孩子
            childNode->setLChild(temNode);
            temNode->setRChild(nullptr);
        }else{  //如果孩子结点有两个孩子结点，则将孩子结点的左孩子移到双亲结点的右下角
            temNode->setRChild(childNode->getLChild());
            childNode->setLChild(temNode);
        }


    }
    depth = getHeight(root);  //重新计算树的深度和bf
}

void AVLTree::add(string usn, string pwd) {
    add(root, new Node(usn, pwd));
}

void AVLTree::updatePassword(string usn, string pwd, Node *T) {
    if(T == nullptr) return;

    updatePassword(usn, pwd, T->getLChild());

    if(usn == T->getUser().username && pwd == T->getUser().password){
        cout << "        新密码:";
        string p;
        cin>>p;
        T->getUser().password = p;
        sign = true;
        return;
    }

    updatePassword(usn, pwd, T->getRChild());
}

void AVLTree::updatePassword(string usn, string pwd) {
    updatePassword(usn, pwd, root);
}

void AVLTree::save(ofstream &outFile) {
    save(outFile,root);
}

void AVLTree::login(string usn, string pwd, Node *T) {
    if(T == nullptr) return;

    login(usn, pwd, T->getLChild());

    if(usn == T->getUser().username && pwd == T->getUser().password){
        sign = true;
    }
    login(usn, pwd, T->getRChild());
}

void AVLTree::login(string usn, string pwd) {
    login(usn, pwd, root);
}

bool AVLTree::getSign() {
    return sign;
}

void AVLTree::reSign() {
    sign = false;
}

void AVLTree::save(ofstream &outFile, Node *T) {
    if(T == nullptr) return;
    if(T == root) outFile<<T->getUser().username<<'@'<<T->getUser().password;
    else outFile<<endl<<T->getUser().username<<'@'<<T->getUser().password;
    save(outFile, T->getLChild());
    save(outFile, T->getRChild());
}


