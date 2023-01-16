package com.control;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

import jakarta.servlet.RequestDispatcher;
import jakarta.servlet.ServletException;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;

@WebServlet("/LoginServlet.do")
public class LoginServlet extends HttpServlet {
  private String message = "Unknow error";
  private String code = "200500";

  @Override
  public void init() throws ServletException {
    this.message = "Unknow error";
    this.code = "200500";
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {
    PrintWriter out = res.getWriter();
    try {
      String email = req.getParameter("email");
      String password = req.getParameter("password");

      String filePath = "/Users/j10c/projects/java/bank/target/bank/WEB-INF" + "/customerinfo.txt";
      File file = new File(filePath);

      FileReader fr = new FileReader(file);
      BufferedReader br = new BufferedReader(fr);

      boolean isValid = false;
      while (br.ready()) {
        String item = br.readLine();
        if (item.split(" ")[0].equals(email) && item.split(" ")[1].equals(password)) {
          isValid = true;
          break;
        }
      }

      if (isValid) {
        RequestDispatcher rd = req.getRequestDispatcher("/welcome.jsp");
        HttpSession session = req.getSession();
        session.setAttribute(filePath, br);
        session.setAttribute("email", email);
        rd.forward(req, res);
      } else {
        RequestDispatcher rd = req.getRequestDispatcher("/loginFail.jsp");
        rd.forward(req, res);
      }

      br.close();
      fr.close();

      this.message = "ok";
      this.code = "200";
    } catch (Exception err) {
      err.printStackTrace();
      this.message = err.toString();
    }
    out.println(this.message);
    out.println(this.code);
  }
}
