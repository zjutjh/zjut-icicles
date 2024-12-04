package cn.edu.zjut.service;

public class UserService implements IUserService
{

    @Override
    public boolean login(String username, String password)
    {
        // 简化的登录逻辑，判断用户名和密码是否相同
        return username != null && password != null && username.equals(password);
    }
}
