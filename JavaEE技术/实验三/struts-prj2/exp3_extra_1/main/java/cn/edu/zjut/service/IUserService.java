package cn.edu.zjut.service;

import cn.edu.zjut.bean.UserBean;

public interface IUserService
{
    public boolean login(UserBean loginUser);
    public boolean register(UserBean loginUser);
}
