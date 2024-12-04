package cn.edu.zjut;

import java.io.*;
import javax.servlet.*;
import javax.servlet.http.*;

import cn.edu.zjut.model.*;

public class LoginController extends HttpServlet
{
    public void doPost(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException
    {
        String username = request.getParameter("username");
        String password = request.getParameter("password");
        UserBean user = new UserBean();
        user.setUsername(username);
        user.setPassword(password);
        if (checkUser(user))
        {
            request.setAttribute("USER", user);
            RequestDispatcher dispatcher = request.getRequestDispatcher("/loginSuccess.jsp");
            dispatcher.forward(request, response);
        }
        else
        {
            response.sendRedirect("/javaweb-prj1/loginFailed.jsp");
        }
    }

    boolean checkUser(UserBean user)
    {
        if ("zjut".equals(user.getUsername()) && "zjut".equals(user.getPassword()) && "管理员".equals(user.getType()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}