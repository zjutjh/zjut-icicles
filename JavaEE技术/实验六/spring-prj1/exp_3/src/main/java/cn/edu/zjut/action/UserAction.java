package cn.edu.zjut.action;

import cn.edu.zjut.po.Customer;
import cn.edu.zjut.service.IUserService;

public class UserAction
{
    private Customer loginUser;
    private IUserService userService = null;

    public void setUserService(IUserService userService)
    {
        this.userService = userService;
    }

    public Customer getLoginUser()
    {
        return loginUser;
    }

    public void setLoginUser(Customer loginUser)
    {
        this.loginUser = loginUser;
    }

    public String execute()
    {
        userService.saveUser(loginUser);
        return "success";
    }
}
