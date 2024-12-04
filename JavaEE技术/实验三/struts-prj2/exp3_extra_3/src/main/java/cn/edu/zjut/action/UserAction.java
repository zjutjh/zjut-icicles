package cn.edu.zjut.action;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.service.UserService;
import com.opensymphony.xwork2.ActionContext;

import java.util.Map;

public class UserAction
{
    private UserBean loginUser;
    private Map session;
    private UserService userService = new UserService();

    public void setUserService(UserService userService)
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

//    public String login()
//    {
//        if (userService.login(loginUser))
//        {
//            return "success";
//        }
//        else
//        {
//            return "fail";
//        }
//    }

    public String login()
            throws Exception
    {
        ActionContext ctx = ActionContext.getContext();
        session = (Map) ctx.getSession();
        UserService userServ = new UserService();
        try
        {
            if (userServ.login(loginUser))
            {
                session.put("user", loginUser.getUsername());
                return "success";
            }
            else
            {
                return "fail";
            }
        }
        catch (Exception e)
        {
//            e.printStackTrace();
//            return "exception";
            throw e;
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
}


