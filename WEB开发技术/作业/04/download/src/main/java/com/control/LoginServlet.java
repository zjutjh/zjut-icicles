package com.control;

import jakarta.servlet.http.*;

import java.io.IOException;
import java.io.PrintWriter;

import jakarta.servlet.*;
import jakarta.servlet.annotation.WebServlet;

@WebServlet("/LoginServlet.do")
public class LoginServlet extends HttpServlet {
  private String defaultUsername = "admin";
  private String defaultPassword = "12345678a";

  @Override
  public void init() throws ServletException {
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    PrintWriter out = resp.getWriter();

    if (req.getParameter("username").equals(this.defaultUsername) &&
        req.getParameter("password").equals(this.defaultPassword)) {
      RequestDispatcher rd = req.getRequestDispatcher("FileDownloadServlet.do");
      HttpSession session = req.getSession();
      session.setAttribute("username", req.getParameter("username"));
      session.setAttribute("password", req.getParameter("password"));
      rd.forward(req, resp);
    }
    out.println("wrong username or password!");
  }

  @Override
  public void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    doPost(req, resp);
  }

}
