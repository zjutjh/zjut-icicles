package com.control;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import com.model.Customer;

import jakarta.servlet.RequestDispatcher;
import jakarta.servlet.ServletException;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;

@WebServlet("/getAllCustomer.do")
public class AllCustomerServlet extends HttpServlet {
  private String message;
  private String code;
  private ArrayList<Customer> customers = new ArrayList<Customer>();

  public void init() {
    this.customers.clear();
    this.message = "Unknow Error";
    this.code = "200500";
  }

  @Override
  public void doGet(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {
    this.init();
    PrintWriter out = res.getWriter();
    try {
      RequestDispatcher rd = req.getRequestDispatcher("/displayAllCustomer.jsp");
      HttpSession session = req.getSession();

      String filePath = "/Users/j10c/projects/java/bank/target/bank/WEB-INF" + "/customerinfo.txt";
      File file = new File(filePath);
      FileReader fr = new FileReader(file);
      BufferedReader br = new BufferedReader(fr);

      while (br.ready()) {
        String item = br.readLine();
        this.customers.add(new Customer());
        Customer lastCustomer = this.customers.get(this.customers.size() - 1);
        lastCustomer.setEmail(item.split(" ")[0]);
        lastCustomer.setPassword(item.split(" ")[1]);
        lastCustomer.setCustName(item.split(" ")[2]);
        lastCustomer.setPhone(item.split(" ")[3]);
      }
      br.close();
      fr.close();

      session.setAttribute("customers", this.customers);
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
