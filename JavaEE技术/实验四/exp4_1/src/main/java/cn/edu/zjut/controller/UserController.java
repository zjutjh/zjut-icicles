package cn.edu.zjut.controller;

import cn.edu.zjut.service.IUserService;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.springframework.web.servlet.ModelAndView;
import org.springframework.web.servlet.mvc.Controller;

public class UserController implements Controller
{
    private IUserService userServ;

    public void setUserServ(IUserService userServ)
    {
        this.userServ = userServ;
    }

    public ModelAndView handleRequest(HttpServletRequest arg0, HttpServletResponse arg1)
            throws Exception
    {
//        return new ModelAndView("/loginSuccess.jsp");
        return new ModelAndView("loginSuccess");
    }
}