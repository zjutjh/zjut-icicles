package com.control;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import com.model.Customer;

import jakarta.servlet.RequestDispatcher;
import jakarta.servlet.ServletException;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;

@WebServlet("/CustomerServlet.do")
public class CustomerServlet extends HttpServlet {
  private String message;
  private String code;
  private Customer customer = new Customer();

  public void init() {
    this.message = "Unknow Error";
    this.code = "200500";
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {

    PrintWriter out = res.getWriter();
    try {
      RequestDispatcher rd = req.getRequestDispatcher("/displayCustomer.jsp");
      HttpSession session = req.getSession();

      this.customer.setEmail(req.getParameter("email"));
      this.customer.setPassword(req.getParameter("password"));
      this.customer.setCustName(req.getParameter("custName"));
      this.customer.setPhone(req.getParameter("phone"));

      String filePath = "/Users/j10c/projects/java/bank/target/bank/WEB-INF" + "/customerinfo.txt";
      File file = new File(filePath);

      FileWriter fw = new FileWriter(file, true);
      BufferedWriter bw = new BufferedWriter(fw);
      bw.write(this.customer.toString());
      bw.newLine();
      bw.flush();
      bw.close();
      fw.close();

      session.setAttribute("customer", this.customer);
      rd.forward(req, res);

      this.message = "ok";
      this.code = "200";
    } catch (Exception err) {
      this.message = err.getMessage();
    }
    out.println(this.message);
    out.println(this.code);
  }
}
