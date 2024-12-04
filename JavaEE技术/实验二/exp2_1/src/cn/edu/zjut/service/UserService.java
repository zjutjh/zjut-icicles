package cn.edu.zjut.service;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.dao.IUserDAO;

public class UserService implements IUserService
{
    private IUserDAO userDAO = null;

    public UserService()
    {
        System.out.println("create UserService.");
    }

    public void setUserDAO(IUserDAO userDAO)
    {
        System.out.println("--setUserDAO--");
        this.userDAO = userDAO;
    }

    public void login(UserBean user)
    {
        System.out.println("execute --login()-- method.");
        userDAO.search(user);
    }
}