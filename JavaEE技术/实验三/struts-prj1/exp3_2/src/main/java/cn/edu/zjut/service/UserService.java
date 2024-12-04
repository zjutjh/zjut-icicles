package cn.edu.zjut.service;

import cn.edu.zjut.bean.UserBean;

public class UserService implements IUserService
{
    public boolean login(UserBean loginUser)
    {
        if (loginUser.getUsername().equals(loginUser.getPassword()))
        {
            return true;
        }
        return false;
    }

    public boolean register(UserBean loginUser)
    {
        if(loginUser.getUsername().equals(loginUser.getPassword())
                && loginUser.getUsername().equals(loginUser.getRepassword()))
        {
            return true;
        }
        return false;
    }
}