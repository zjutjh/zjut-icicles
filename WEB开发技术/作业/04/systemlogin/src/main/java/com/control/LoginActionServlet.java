package com.control;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

import com.model.User;

import jakarta.servlet.*;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.*;

@WebServlet("/LoginActionServlet.do")
public class LoginActionServlet extends HttpServlet {

  private String filePath = "/WEB-INF/user.txt";

  @Override
  public void init() throws ServletException {

  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    String identity = req.getParameter("identity");
    String username = req.getParameter("username");
    String password = req.getParameter("password");

    // 加不加无所谓，要在jsp中加
    // resp.setContentType("text/html;charset=utf-8");
    PrintWriter out = resp.getWriter();
    RequestDispatcher rd = null;
    File file = new File(this.getServletContext().getRealPath("") + this.filePath);
    FileReader fr = new FileReader(file);
    BufferedReader br = new BufferedReader(fr);
    String validName = null;

    try {

      while (br.ready()) {
        String item = br.readLine();
        String[] args = item.split("\t");
        if (username.equals(args[0]) && password.equals(args[1])) {
          if (identity.equals("teacher") && args[3].equals("教师") ||
              identity.equals("student") && args[3].equals("学生")) {
            rd = req.getRequestDispatcher(identity + ".jsp");
            validName = args[2];
            break;
          }
        }
      }
      if (validName == null) {
        rd = req.getRequestDispatcher("login.jsp");
        rd.forward(req, resp);
      } else {
        req.getSession().setAttribute("user", new User(username, password, validName, identity));
      }

      rd.forward(req, resp);

    } catch (Exception err) {
      rd = req.getRequestDispatcher("login.jsp");
      rd.forward(req, resp);
      err.printStackTrace();
      out.println(err.getMessage());
    } finally {
      br.close();
      fr.close();
    }
  }

  @Override
  public void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    doPost(req, resp);
  }
}
