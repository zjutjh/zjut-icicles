package com.control;

import jakarta.servlet.http.*;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;

import jakarta.servlet.*;
import jakarta.servlet.annotation.WebServlet;

@WebServlet("/FileDownloadServlet.do")
public class FileDownloadServlet extends HttpServlet {
  private String defaultUsername = "admin";
  private String defaultPassword = "12345678a";

  @Override
  public void init() throws ServletException {
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    PrintWriter out = resp.getWriter();
    HttpSession session = req.getSession();
    String username = (String) session.getAttribute("username");
    String password = (String) session.getAttribute("password");

    if (!username.equals(this.defaultUsername) ||
        !password.equals(this.defaultPassword)) {
      RequestDispatcher rd = req.getRequestDispatcher("login.jsp");
      rd.forward(req, resp);
    } else {
      String filePath = "/WEB-INF/files";
      // File file = new File(filePath + "/servlet.pdf");
      resp.setHeader("Content-Disposition", "attachment;filename=" + "servlet.pdf");
      resp.setContentType("application/pdf");

      FileInputStream fis = new FileInputStream(this.getServletContext().getRealPath("") + filePath + "/servlet.pdf");
      int i = 0;
      while ((i = fis.read()) != -1) {
        out.write(i);
      }
      out.close();
      fis.close();
    }

  }

  @Override
  public void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    doPost(req, resp);
  }
}
