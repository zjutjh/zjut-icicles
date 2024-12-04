package cn.edu.zjut.action;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.service.UserService;
import cn.edu.zjut.service.IUserService;

public class UserAction
{
    private UserBean loginUser;
    private IUserService userService = null;

    public void setUserService(IUserService userService)
    {
        this.userService = userService;
    }

    public UserBean getLoginUser()
    {
        return loginUser;
    }

    public void setLoginUser(UserBean loginUser)
    {
        this.loginUser = loginUser;
    }

    public String execute()
    {
//        if (userService.login(loginUser))
//        {
//            return "success";
//        }
//        return "fail";
        if (userService.register(loginUser))
        {
            return "success";
        }
        return "fail";
    }
}