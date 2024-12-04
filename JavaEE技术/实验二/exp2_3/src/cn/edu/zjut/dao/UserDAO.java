package cn.edu.zjut.dao;

import cn.edu.zjut.bean.UserBean;

public class UserDAO implements IUserDAO
{
    public UserDAO()
    {
        System.out.println("create UserDAO.");
    }

    public void search(UserBean user)
    {
        System.out.println("execute --search()-- method.");
    }
}