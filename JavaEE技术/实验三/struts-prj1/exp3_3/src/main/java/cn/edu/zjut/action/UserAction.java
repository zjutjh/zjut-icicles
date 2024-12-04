package cn.edu.zjut.action;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.service.UserService;
import cn.edu.zjut.service.IUserService;
import com.opensymphony.xwork2.ActionSupport;

public class UserAction extends ActionSupport
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

    public String login()
    {
        if (userService.login(loginUser))
        {
            this.addActionMessage("登录成功！！！！！！");
            return "success";
        }
        else
        {
            this.addActionError("用户名或密码错误，请重新输入！");
            return "fail";
        }
    }

    public String register()
    {
        if (userService.register(loginUser))
        {
            return "regsuccess";
        }
        return "regfail";
    }

    public void validateLogin()
    {
        String username = loginUser.getUsername();
        String pwd = loginUser.getPassword();
        if (username == null || username.equals(""))
        {
            this.addFieldError("loginUser.username", "请输入您的用户名！");
        }
        if (pwd == null || pwd.equals(""))
        {
            this.addFieldError("loginUser.password", "请输入您的密码！");
        }
    }
}


