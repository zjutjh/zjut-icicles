package cn.edu.zjut;

import java.io.*;
import javax.servlet.*;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.*;

import cn.edu.zjut.dao.UserDAO;
import cn.edu.zjut.model.UserBean;

@WebServlet("/RegisterServlet")
public class RegisterController extends HttpServlet
{
    protected void doPost(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException
    {
        String username = request.getParameter("username");
        String password = request.getParameter("password");
        int type = Integer.parseInt(request.getParameter("usertype"));

        UserBean user = new UserBean();
        user.setUsername(username);
        user.setPassword(password);
        user.setType(type);

        UserDAO userDao = new UserDAO();
        boolean inserted = userDao.insert(user);

        if (inserted)
        {
            // 注册成功，跳转到注册成功页面或其他页面
            response.sendRedirect("RegisterSuccess.jsp");
        }
        else
        {
            // 注册失败，跳转到注册失败页面或其他页面
            response.sendRedirect("RegisterFail.jsp");
        }
    }
}
