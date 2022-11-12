#include "AVLTree.h"
int main() {
    ifstream inFile;
    ofstream outFile;
    int num = -1, num2 = -1;
    string name, pwd;

    inFile.open("test.txt",ios::in);
    outFile.open("test1.txt");
    AVLTree test(inFile);  //读取数据

    while(true){
        system("cls");
        cout << "--------用户登录系统--------\n\n";
        cout << "        1:用户登录\n";
        cout << "        0:退出\n\n";
        cout << "----------------------------\n\n";
        cout << "        请输入您的选择:";
        cin >> num;
        switch (num)
        {
            case 0:
                break;
            case 1:
            {
                system("cls");
                cout << "----------登录----------\n\n";
                cout << "        用户名：";
                cin >> name;
                cout << "\n        密码：";
                cin >> pwd;
                test.login(name, pwd);
                if (test.getSign())
                {
                    test.reSign();
                    while (1)
                    {
                        system("cls");
                        cout << "--------登录成功!---------\n\n";
                        cout << "        1:用户密码更新\n";
                        cout << "        2:用户添加\n";
                        cout << "        3:用户删除\n";
                        cout << "        4:显示当前树\n";
                        cout << "        0:退出\n\n";
                        cout << "--------------------------\n\n";
                        cout << "        请输入您的选择:";
                        cin >> num2;
                        switch (num2)
                        {
                            case 0:
                                break;
                            case 1:
                            {
                                system("cls");
                                cout << "--------用户密码更新--------\n\n";
                                test.updatePassword(name, pwd);
                                if (test.getSign())
                                {
                                    test.reSign();
                                    cout << "\n        密码修改成功!\n";
                                    cout << "--------------------------\n\n";
                                }
                                system("pause");
                                break;
                            }
                            case 2:
                            {
                                system("cls");
                                string nName, nPwd;
                                cout << "--------用户添加前--------\n";
                                test.print();
                                cout << "--------------------------\n\n";
                                cout << "---------用户添加---------\n\n";
                                cout << "         用户名：";
                                cin >> nName;
                                cout << "\n         密码：";
                                cin >> nPwd;
                                test.add(nName, nPwd);
                                if (test.getSign())
                                {
                                    test.reSign();
                                    cout << "\n        用户添加成功!\n";
                                    cout << "--------------------------\n\n";
                                    cout << "--------用户添加后--------\n";
                                    test.print();
                                    cout << "--------------------------\n\n";
                                }
                                system("pause");
                                break;
                            }
                            case 3:
                            {
                                system("cls");
                                string dName;
                                cout << "--------用户删除前--------\n";
                                test.print();
                                cout << "--------------------------\n\n";
                                cout << "---------用户删除---------\n\n";
                                cout << "        删除的用户名:";
                                cin >> dName;
                                test.deleteNode(dName);
                                if (test.getSign()) {
                                    test.reSign();
                                    cout << "\n        用户删除成功!\n\n";
                                    cout << "--------用户删除后--------\n";
                                    test.print();
                                    cout << "--------------------------\n\n";
                                }
                                else
                                {
                                    cout << "\n        用户删除失败!\n\n";
                                }
                                system("pause");
                                break;
                            }
                            case 4:
                                system("cls");
                                test.print();
                                system("pause");
                                break;
                            default:
                                break;
                        }
                        if (num2 == 0)
                        {
                            system("cls");
                            break;
                        }
                    }
                }
                else
                {
                    cout << "\n     用户名或密码错误!\n\n";
                    system("pause");
                }
                break;
            }
            default:
                break;
        }
        if (num == 0)break;
    }
    test.save(outFile);  //保存数据
    outFile.close();
    inFile.close();
    return 0;
}
