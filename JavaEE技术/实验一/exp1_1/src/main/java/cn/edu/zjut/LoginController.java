package cn.edu.zjut;

import java.io.*;
import javax.servlet.*;
import javax.servlet.http.*;

public class LoginController extends HttpServlet
{
    protected void doPost(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException
    {
        response.setContentType("text/html;charset=utf-8");
        PrintWriter out = response.getWriter();
        String username = request.getParameter("username");
        String password = request.getParameter("password");
        if ("zjut".equals(username) && "zjut".equals(password))
        {
            out.println("登录成功，欢迎您！");
        }
        else
        {
            out.println("用户名或密码错误！");
        }
    }
}