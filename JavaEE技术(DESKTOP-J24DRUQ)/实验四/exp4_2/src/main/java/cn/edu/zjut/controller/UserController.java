package cn.edu.zjut.controller;

import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.service.IUserService;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.ui.Model;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import org.springframework.beans.factory.annotation.Autowired;

import javax.servlet.http.HttpServletRequest;

@Controller
public class UserController
{
    @Autowired
    @Qualifier("userServ")
    private IUserService userServ;

//    public void setUserServ(IUserService userServ)
//    {
//        this.userServ = userServ;
//    }

    @RequestMapping(value = "/login/{uname}/{upass}", method = RequestMethod.GET)
    public String login(@PathVariable String uname, @PathVariable String upass, Model model)
    {
        if (uname.equals(upass))
        {
            model.addAttribute("uname", uname);
            return "loginSuccess";
        }
        else
            return "login";
    }
}