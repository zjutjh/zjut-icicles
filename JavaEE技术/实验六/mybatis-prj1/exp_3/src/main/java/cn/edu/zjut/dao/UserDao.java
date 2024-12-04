package cn.edu.zjut.dao;

import cn.edu.zjut.pojo.User;

public interface UserDao
{
    // 根据用户ID查询用户信息
    User getUserById(int id);
}
