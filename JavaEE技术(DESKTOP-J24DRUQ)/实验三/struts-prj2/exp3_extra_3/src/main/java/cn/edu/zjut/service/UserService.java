package cn.edu.zjut.service;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.exception.UserException;

public class UserService implements IUserService
{
    public boolean login(UserBean loginUser)
            throws Exception
    {
        if (loginUser.getUsername().equalsIgnoreCase("admin"))
        {
            throw new UserException("用户名不能为 admin");
        }
        if (loginUser.getPassword().toUpperCase().contains(" AND ") || loginUser.getPassword().toUpperCase().contains(" OR "))
        {
            throw new java.sql.SQLException("密码不能包括' and '或' or '");
        }
        if (loginUser.getUsername().equals(loginUser.getPassword()))
        {
            return true;
        }
        else
            return false;
    }

    public boolean register(UserBean loginUser)
    {
        if (loginUser.getUsername().equals(loginUser.getPassword()) && loginUser.getUsername().equals(loginUser.getRepassword()))
        {
            return true;
        }
        return false;
    }
}