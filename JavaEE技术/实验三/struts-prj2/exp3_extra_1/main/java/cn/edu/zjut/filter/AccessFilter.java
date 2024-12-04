package cn.edu.zjut.filter;

import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.ServletRequest;
import jakarta.servlet.ServletResponse;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;

import java.io.IOException;

public class AccessFilter
{
    public void doFilter(ServletRequest arg0, ServletResponse arg1, FilterChain filterChain)
            throws IOException, ServletException
    {
        System.out.println("Access Filter executed!");
        HttpServletRequest request = (HttpServletRequest) arg0;
        HttpServletResponse response = (HttpServletResponse) arg1;
        HttpSession session = request.getSession();
        if (session.getAttribute("user") == null && request.getRequestURI().indexOf("login.jsp") == -1 && request.getRequestURI().indexOf("register.jsp") == -1)
        {
            response.sendRedirect("login.jsp");
            return;
        }
        filterChain.doFilter(arg0, arg1);
    }
}
